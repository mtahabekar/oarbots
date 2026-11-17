#!/usr/bin/env python3
"""Prepare a robot's fabric CBF bridge to execute the simulated cell's fabric twin commands.

The twin of the simulated cell commands grip velocities in the sim cell's world frame. To execute
them on the real robot, the bridge uses the sim layout (world -> base_link and end_effector ->
fabric_grip from fabric_sim_cell's calibration_exact.yaml) as if it were this lab's calibration.
That makes a world-frame command move the real arm the same way, in its own frame, as its
simulated copy. It is a mirror for directions, not a measurement of the lab.

This step checks that the arm is at the SRDF Home state and that nothing else commands it, sets
the bridge's workspace box around the tool's Home position (world frame), checks with MoveIt that
every corner, edge and face centre of the box is reachable without collision or a configuration
flip, and writes the bridge parameters. `fabric_session.sh start-bridge` runs it, then starts
`sim_bridge.launch.py` with the result.
    python3 sim_bridge.py --robot silver [--box 0.15 0.15 0.12] [--out FILE]
"""
import argparse
import math
import sys
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.time import Time
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from first_motion import HOME_TOL_RAD, srdf_home, wrap  # noqa: E402

# The Jaco2's rated payload, 1.6 kg at mid-range: the bridge's force stop for this session. The raw
# wrist force swings +/-4 N (peak 10 N) when the arm jerks into motion (measured 2026-11-16), and the
# bridge does not filter, so the cell's 6 N placeholder would stop it on vibration alone.
FORCE_LIMIT_N = 15.7
# Workspace box per robot (world frame of the sim layout; negative-side and positive-side extents in m),
# the largest that MoveIt found reachable at all 26 points from Home on 2026-11-16. Each robot's outward
# side (Silver +x, Blue -x) is short: the forearm and wrist camera approach the UWB wing there, and
# going outward and back at once flips the wrist.
BOXES = {'silver': ((0.12, 0.05, 0.10), (0.04, 0.12, 0.08)),
         'blue': ((0.04, 0.05, 0.10), (0.12, 0.12, 0.08))}


def sim_layout(robot):
    path = Path(get_package_share_directory('fabric_sim_cell')) / 'config' / 'calibration_exact.yaml'
    calib = yaml.safe_load(path.read_text())
    find = lambda parent, child: next(t for t in calib['transforms'] if t['parent'] == parent and t['child'] == child)  # noqa: E731
    return (find(calib['world_frame'], f'oarbot_{robot}/base_link'),
            find(f'oarbot_{robot}/j2n6s300_end_effector', f'oarbot_{robot}/fabric_grip'))


def rotation(q):
    x, y, z, w = q
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def apply(r, v):
    return [sum(r[i][j] * v[j] for j in range(3)) for i in range(3)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robot', choices=('blue', 'silver'), required=True)
    parser.add_argument('--box', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='extents in metres of the workspace box around the Home tool position (world frame); '
                             'default: the measured per-robot box (BOXES)')
    parser.add_argument('--box-min', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='negative-side extents (positive numbers) if the box is not symmetric; --box is then the positive side')
    parser.add_argument('--max-speed', type=float, default=0.08)
    parser.add_argument('--out', default=str(Path.home() / 'fabric_ws' / 'run' / 'sim_bridge.yaml'))
    args = parser.parse_args()

    robot, ns, prefix = args.robot, f'/oarbot_{args.robot}', f'oarbot_{args.robot}/'
    base, ee = prefix + 'base_link', prefix + 'j2n6s300_end_effector'
    joints = [prefix + f'j2n6s300_joint_{i}' for i in range(1, 7)]
    world_base, ee_grip = sim_layout(robot)
    if math.dist(ee_grip['translation'], (0, 0, 0)) > 1e-6:
        sys.exit('This step assumes fabric_grip at the end effector, as in the sim layout')

    rclpy.init()
    node = rclpy.create_node(f'sim_bridge_prepare_{robot}')
    tf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf, node)
    state = {}
    node.create_subscription(JointState, ns + '/joint_states', lambda m: state.__setitem__('js', m), 10)
    end = time.monotonic() + 12.0
    while ('js' not in state or not tf.can_transform(base, ee, Time())) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if 'js' not in state:
        sys.exit('No joint states or TF; is the fabric session stack running?')
    end = time.monotonic() + 2.0
    while time.monotonic() < end:   # discovery, before counting publishers
        rclpy.spin_once(node, timeout_sec=0.05)

    js = state['js']
    pos = dict(zip(js.name, js.position))
    home = srdf_home(robot)
    offset = max(abs(wrap(pos[n] - home[n])) for n in joints)
    if offset > HOME_TOL_RAD:
        sys.exit(f'Arm is not at Home (largest joint offset {offset:.3f} rad); home it with its joystick first')
    guard = [p.node_name for p in node.get_publishers_info_by_topic(ns + '/kinova/j2n6s300_driver/in/cartesian_velocity')]
    if guard != ['command_guard']:
        sys.exit(f'Driver input must have only command_guard as publisher, found {guard}')
    if node.count_publishers(ns + '/commands/fabric_cbf/arm'):
        sys.exit(f'{ns}/commands/fabric_cbf/arm already has a publisher (another bridge or script?)')
    has_force = node.count_publishers(ns + '/force_torque_calibrated') > 0

    t = tf.lookup_transform(base, ee, Time()).transform
    tool_base = [t.translation.x, t.translation.y, t.translation.z]
    r_wb = rotation(world_base['rotation_xyzw'])
    t_wb = world_base['translation']
    tool_world = [a + b for a, b in zip(apply(r_wb, tool_base), t_wb)]
    r_bw = [list(row) for row in zip(*r_wb)]

    ik = node.create_client(GetPositionIK, ns + '/compute_ik')
    validity = node.create_client(GetStateValidity, ns + '/check_state_validity')
    for client in (ik, validity):
        if not client.wait_for_service(timeout_sec=5.0):
            sys.exit(f'{client.srv_name} is not available')

    def call(client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done():
            sys.exit(f'{client.srv_name} did not answer')
        return future.result()

    req = GetStateValidity.Request()
    req.robot_state, req.group_name = RobotState(joint_state=js), 'arm'
    if not call(validity, req).valid:
        sys.exit('MoveIt reports the current state in collision')
    if args.box:
        high = list(args.box)
        low = list(args.box_min) if args.box_min else list(args.box)
    else:
        low, high = (list(v) for v in BOXES[robot])
    failures = []
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            for k in (-1, 0, 1):
                if (i, j, k) == (0, 0, 0):
                    continue
                offset_world = [(high if s > 0 else low)[n] * s for n, s in enumerate((i, j, k))]
                target_base = apply(r_bw, [tool_world[n] + offset_world[n] - t_wb[n] for n in range(3)])
                req = GetPositionIK.Request()
                r = req.ik_request
                r.group_name, r.avoid_collisions, r.ik_link_name = 'arm', True, ee
                r.robot_state = RobotState(joint_state=js)
                r.timeout = DurationMsg(sec=0, nanosec=200_000_000)
                pose = PoseStamped()
                pose.header.frame_id = base
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target_base
                pose.pose.orientation = t.rotation
                r.pose_stamped = pose
                res = call(ik, req)
                where = [round(v * 100, 1) for v in offset_world]
                if res.error_code.val != 1:
                    failures.append(f'{where} cm: no collision-free IK')
                    continue
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                change = max(abs(wrap(sol[n] - pos[n])) for n in joints)
                if change > 0.3 + 10 * math.dist((0, 0, 0), offset_world):
                    failures.append(f'{where} cm: configuration flip ({change:.2f} rad)')
    if failures:
        sys.exit('Workspace box (world frame) has unreachable points; shrink those sides:\n  ' + '\n  '.join(failures))

    params = {'/**': {'ros__parameters': {
        'world_frame': 'world', 'command_frame': prefix + 'j2n6s300_link_base', 'tool_frame': prefix + 'fabric_grip',
        'max_speed_mps': args.max_speed, 'command_timeout_sec': 0.15, 'force_limit_n': FORCE_LIMIT_N,
        'force_timeout_sec': 0.25, 'pose_timeout_sec': 0.25, 'require_force': has_force,
        'workspace_min': [tool_world[n] - low[n] for n in range(3)],
        'workspace_max': [tool_world[n] + high[n] for n in range(3)]}}}
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(params, sort_keys=False))
    print(f'{robot}: Home offset {offset:.3f} rad, 26 box points reachable, tool at {[round(v, 3) for v in tool_world]} m '
          f'(sim world), box -{[round(b * 100) for b in low]} / +{[round(b * 100) for b in high]} cm, wrist force check {"on" if has_force else "off (no sensor)"}. '
          f'Parameters: {out}', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
