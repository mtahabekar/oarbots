#!/usr/bin/env python3
"""Prepare a narrower warm-up pose from grasp.yaml; read-only ROS/MoveIt queries, no arm commands.

    source scripts/hw/laptop_env.sh
    python3 scripts/hw/warmup_pose.py --report <session_dir>

Uses the deployed URDF, the bridge's differential kinematics and the current MoveIt scene. Preserves
each grip's height and orientation, and checks both the approach and horizontal warm-up excursions.
"""
import argparse
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CELL = REPO / 'experiments/hardware/cell'
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / 'src/oarbot_runtime'))
from oarbot_runtime.arm_kinematics import Chain, joint_rates, rotation_vector  # noqa: E402

ROBOTS = ('blue', 'silver')
LINKS = ('j2n6s300_end_effector', 'j2n6s300_link_5', 'j2n6s300_link_4', 'camera_base')


def solve(chain, seed, target):
    q = np.array(seed, float)
    for _ in range(100):
        J, actual = chain.jacobian(q)
        error = np.r_[target[:3, 3] - actual[:3, 3], rotation_vector(target[:3, :3] @ actual[:3, :3].T)]
        if np.linalg.norm(error[:3]) < 1e-5 and np.linalg.norm(error[3:]) < 1e-4:
            return q
        step = joint_rates(J, error, .005)
        q += step * min(1., math.radians(2) / max(np.max(abs(step)), 1e-12))
    raise ValueError('Could not reach the warm-up point on the seeded arm branch')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--span', type=float, default=.77, help='robot grip separation (m), default 0.77')
    ap.add_argument('--output', type=Path, default=CELL / 'warmup_grasp.yaml')
    ap.add_argument('--report', type=Path, required=True)
    args = ap.parse_args()
    config = yaml.safe_load((CELL / 'twin.yaml').read_text())
    guard = config.get('stretch_guard', {})
    limit = config['sheet']['width_m'] - guard.get('edge_slack_m', .03)
    if not 0 < args.span <= limit - .025:
        ap.error(f'Keep at least 25 mm reserve below the stretch-guard limit ({limit:.3f} m)')
    grasp = yaml.safe_load((CELL / 'grasp.yaml').read_text())
    calibration = yaml.safe_load((CELL / 'calibration.yaml').read_text())
    # This generator keeps the present zero clip-offset convention explicit. Re-measure the
    # pose after calibrating the clip offsets rather than silently using the wrong tool point.
    for row in calibration['transforms']:
        if row['child'].endswith('/fabric_grip'):
            if not np.allclose(row['translation'], 0) or not np.allclose(row['rotation_xyzw'], [0, 0, 0, 1]):
                ap.error('Clip offsets changed: extend this generator to solve the calibrated grip transform')

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from urdf_parser_py.urdf import URDF
    from grasp_target import Target, base_poses, path

    args.report.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    nodes, models = {}, {}
    try:
        for robot in ROBOTS:
            node = nodes[robot] = Target(robot)
            descriptions = []
            node.create_subscription(String, node.ns + '/robot_description', lambda m, d=descriptions: d.append(m.data),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            node.wait(lambda: descriptions and node.joints is not None, 'joint states and robot_description', sec=30)
            (args.report / f'{robot}.urdf').write_text(descriptions[-1])
            urdf = URDF.from_xml_string(descriptions[-1])
            prefix = f'oarbot_{robot}/'
            chain = Chain(urdf, prefix + 'j2n6s300_link_base', prefix + 'j2n6s300_end_effector')
            seed = np.array(grasp[robot])
            world = base_poses()[robot]
            arm_world = world @ node.links(seed, ['j2n6s300_link_base'])['j2n6s300_link_base']
            models[robot] = dict(chain=chain, urdf=urdf, seed=seed, world=world, arm_world=arm_world,
                                 original=arm_world @ chain.fk(seed), current=np.array(node.current()))
        points = np.array([models[r]['original'][:3, 3] for r in ROBOTS])
        across = points[1] - points[0]
        across[2] = 0
        across /= np.linalg.norm(across)
        # Different original heights remain fixed; solve the horizontal span to obtain the
        # requested full 3D grip separation, not just its x projection.
        horizontal = math.sqrt(args.span**2 - (points[1, 2] - points[0, 2])**2)
        midpoint = (points[0] + points[1]) / 2
        targets, clouds, report = {}, {}, dict(commanded_motion=False, guard_span_limit_m=limit,
                                              requested_span_m=args.span, validation={})
        for index, robot in enumerate(ROBOTS):
            m, node = models[robot], nodes[robot]
            target = m['original'].copy()
            target[:2, 3] = midpoint[:2] + (index - .5) * horizontal * across[:2]
            local = np.linalg.inv(m['arm_world']) @ target
            q = targets[robot] = solve(m['chain'], m['seed'], local)
            checks, positions, speeds = 0, [], []

            def check(joints):
                nonlocal checks
                for name, angle in zip(m['chain'].names, joints):
                    joint = m['urdf'].joint_map[name]
                    if joint.type == 'revolute' and joint.limit is not None:
                        if not joint.limit.lower <= angle <= joint.limit.upper:
                            raise ValueError(f'{robot}: joint limit at {name}')
                valid, contacts = node.contacts(list(joints))
                if not valid:
                    raise ValueError(f'{robot}: MoveIt collision: {contacts}')
                links = node.links(list(joints), list(LINKS))
                positions.append([(m['world'] @ links[name])[:3, 3] for name in LINKS])
                J, _ = m['chain'].jacobian(joints)
                speeds.append(float(math.radians(36) / np.max(np.linalg.norm(np.linalg.pinv(J)[:, :3], axis=1))))
                checks += 1

            # Include both the original grasp and the actual current arm pose as approaches.
            for origin in (m['seed'], m['current']):
                steps = max(2, int(np.ceil(np.degrees(np.max(abs(q - origin))))))
                for joints in path(origin, q, steps)[0]:
                    check(joints)
            # Any horizontal 0-4 cm inset, including diagonal directions. The bridges retain
            # their existing workspace, joint-rate, deadman and freshness guards.
            for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
                seed = q.copy()
                for distance in (.02, .04):
                    excursion = target.copy()
                    excursion[:3, 3] += distance * np.array([np.cos(angle), np.sin(angle), 0.])
                    seed = solve(m['chain'], seed, np.linalg.inv(m['arm_world']) @ excursion)
                    check(seed)
            clouds[robot] = np.asarray(positions)
            bridge = yaml.safe_load((CELL / 'bridge.yaml').read_text())
            box = {**bridge['common'], **bridge.get(robot, {})}
            if np.any(clouds[robot][:, 0] < box['workspace_min']) or np.any(clouds[robot][:, 0] > box['workspace_max']):
                raise ValueError(f'{robot}: approach or warm-up exceeds the existing bridge workspace')
            actual = m['arm_world'] @ m['chain'].fk(q)
            report['validation'][robot] = dict(joints_rad=q.tolist(), current_joints_rad=m['current'].tolist(),
                original_joints_rad=m['seed'].tolist(), joint_change_from_grasp_deg=np.degrees(q-m['seed']).tolist(),
                original_grip_world_m=m['original'][:3, 3].tolist(), grip_world_m=actual[:3, 3].tolist(),
                position_error_m=float(np.linalg.norm(actual[:3, 3]-target[:3, 3])),
                orientation_error_deg=float(np.degrees(np.linalg.norm(rotation_vector(actual[:3, :3] @ target[:3, :3].T)))),
                moveit_states_checked=checks, minimum_directional_speed_mps=min(speeds))
        # Check every combination along both approaches and excursions, not just simultaneous
        # equal-time samples. This includes the opposite arm standing at either endpoint.
        minimum = float(min(np.linalg.norm(a-b) for a, b in itertools.product(clouds['blue'].reshape(-1, 3),
                                                                               clouds['silver'].reshape(-1, 3))))
        if minimum < .15:
            raise ValueError(f'Wrist/tool/camera points pass only {minimum:.3f} m from the opposite arm')
        p = [np.array(report['validation'][r]['grip_world_m']) for r in ROBOTS]
        report.update(qualified=True, checked_link_clearance_m=minimum, actual_span_m=float(np.linalg.norm(p[1]-p[0])))
        pose = dict(source='Warm-up pose derived from grasp.yaml with fixed tool orientations and heights',
                    robot_span_m=report['actual_span_m'], guard_span_limit_m=limit,
                    reserve_m=limit-report['actual_span_m'], validation_report=str(args.report / 'validation.json'),
                    **{r: targets[r].tolist() for r in ROBOTS})
        (args.report / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
        args.output.write_text(yaml.safe_dump(pose, sort_keys=False))
        print(json.dumps(report, indent=2))
        print(f'Prepared {args.output}; no robot motion commanded.')
    finally:
        for node in nodes.values():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
