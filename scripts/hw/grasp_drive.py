#!/usr/bin/env python3
"""Drive one arm's tool in a straight line to a world position, translation only (the grasp approach).

The tool keeps its orientation: commands carry zero angular velocity, the only Kinova velocity axes
verified so far (gate H3, translation). Commands go to the command guard's fabric_cbf input, so the
arm moves only while the operator holds the tablet deadman with "Fabric Follow (CBF)" selected for
this arm; releasing it stops the arm, holding it again (after a release and press) resumes.

Run on the robot's helper inside a fabric session (stack started; no bridge, no fabric-cell bridge):
    scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/grasp_drive.py --robot blue --to -0.40 -0.42 1.05 --check-only
    scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/grasp_drive.py --robot blue --to -0.40 -0.42 1.05
    scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/grasp_drive.py --robot blue --home
--to is the tool (j2n6s300_end_effector) position in the cell's world frame
(experiments/hardware/cell/calibration.yaml gives the parked base pose); --home drives back to where
the tool is at the SRDF Home joints. Before sending anything MoveIt must find every centimetre of the
straight path reachable with the current orientation, collision-free and without a configuration
flip. Like first_motion.py, the script aborts only on faults: stale data, the hand off the line or
past the target by 3 cm, hand speed far above the command, the tool turning more than 10 deg, a
joint jump, the guard forwarding something else, or a wrist-force change above the rated payload.
"""
import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from kinova_msgs.msg import PoseVelocity
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from first_motion import (FORCE_CONTACT_N, RATE_HZ, SPEED_FACTOR, SPEED_SLACK_MPS, STALE_SEC,  # noqa: E402
                          Abort, FirstMotion, srdf_home, wrap)

REPO = Path(__file__).resolve().parents[2]
SPEED_MPS = 0.04          # above the deadband (no motion at 1 cm/s, almost none at 2 cm/s; H3)
MIN_SPEED_MPS = 0.03
LAG_SEC = 0.45            # measured 0.35-0.56 s on Blue (2026-11-18): stop early by speed x lag
TOL_M = 0.015
LINE_MARGIN_M = 0.03
TURN_LIMIT_DEG = 10.0
JUMP_RAD = 0.1            # a Jaco2 joint moves at most ~12 mrad per 20 ms sample
WAIT_SEC = 120.0
CORRECTIONS = 2


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]]


def turn_deg(q0, q1):
    dot = abs(q0.x*q1.x + q0.y*q1.y + q0.z*q1.z + q0.w*q1.w)
    return math.degrees(2 * math.acos(min(1.0, dot)))


def world_to_base(robot):
    """world -> <robot>/base_link from the cell calibration: (R, t) with p_base = R^T (p_world - t)."""
    cal = yaml.safe_load((REPO / 'experiments' / 'hardware' / 'cell' / 'calibration.yaml').read_text())
    for t in cal['transforms']:
        if t['parent'] == 'world' and t['child'] == f'oarbot_{robot}/base_link':
            x, y, z, w = t['rotation_xyzw']

            class Q:
                pass
            q = Q()
            q.x, q.y, q.z, q.w = x, y, z, w
            return quat_matrix(q), t['translation']
    raise Abort(f'No world -> oarbot_{robot}/base_link in calibration.yaml')


class GraspDrive(FirstMotion):
    def __init__(self, args):
        args.axes, args.step = 'xyz', 0.0
        super().__init__(args)
        self.fk = self.create_client(GetPositionFK, self.ns + '/compute_fk')
        self.fingers_topic = self.ns + '/kinova/j2n6s300_driver/in/cartesian_velocity_with_fingers'

    def base_to_arm(self, p_base):
        t = self.tf.lookup_transform(self.base, self.robot_base, Time()).transform
        R = quat_matrix(t.rotation)
        v = (t.translation.x, t.translation.y, t.translation.z)
        return tuple(sum(R[r][c] * p_base[c] for c in range(3)) + v[r] for r in range(3))

    def target_in_arm_base(self):
        if self.args.home:
            home = srdf_home(self.args.robot)
            req = GetPositionFK.Request()
            req.header.frame_id = self.base
            req.fk_link_names = [self.ee]
            js = type(self.joint_msg)()
            js.name = list(self.arm_joints)
            js.position = [home[n] for n in self.arm_joints]
            req.robot_state = RobotState(joint_state=js)
            res = self.call(self.fk, req)
            if res.error_code.val != 1:
                raise Abort(f'compute_fk failed for Home (MoveIt code {res.error_code.val})')
            p = res.pose_stamped[0].pose.position
            return (p.x, p.y, p.z), 'Home'
        R, t = world_to_base(self.args.robot)
        d = [self.args.to[i] - t[i] for i in range(3)]
        p_base = tuple(sum(R[c][r] * d[c] for c in range(3)) for r in range(3))
        return self.base_to_arm(p_base), f'world {self.args.to}'

    def open_fingers(self, msg):
        """The joint state with the fingers open: MoveIt counts closed fingertips touching each other (Silver's
        are held closed) as a collision, and the fingers do not move during a translation."""
        js = type(msg)()
        js.header, js.name = msg.header, list(msg.name)
        js.position = [0.0 if 'finger' in n else v for n, v in zip(msg.name, msg.position)]
        return js

    def preflight_path(self, target):
        for topic in (self.guard_topic, self.fingers_topic):
            pubs = [p.node_name for p in self.get_publishers_info_by_topic(topic)]
            if pubs not in ([], ['command_guard']):
                raise Abort(f'{topic} must have only command_guard as publisher, found {pubs}')
        if not [p.node_name for p in self.get_publishers_info_by_topic(self.guard_topic)]:
            raise Abort('command_guard is not publishing to the driver')
        if self.count_publishers(self.command_topic):
            raise Abort(f'{self.command_topic} already has a publisher (is a bridge or fabric-cell running?)')
        req = GetStateValidity.Request()
        req.robot_state = RobotState(joint_state=self.open_fingers(self.joint_msg))
        req.group_name = 'arm'
        res = self.call(self.validity, req)
        if not res.valid:
            raise Abort(f'MoveIt reports the current state invalid: {[(c.contact_body_1, c.contact_body_2) for c in res.contacts]}')
        p, rot, _ = self.hand()
        dist = math.dist(p, target)
        n = max(1, int(math.ceil(dist / 0.01)))
        seed = self.open_fingers(self.joint_msg)
        names = list(self.arm_joints)
        prev = self.joints()
        largest = 0.0
        for k in range(1, n + 1):
            req = GetPositionIK.Request()
            r = req.ik_request
            r.group_name = 'arm'
            r.robot_state = RobotState(joint_state=seed)
            r.avoid_collisions = True
            r.ik_link_name = self.ee
            r.timeout = DurationMsg(sec=0, nanosec=300_000_000)
            ps = PoseStamped()
            ps.header.frame_id = self.base
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (p[i] + (target[i] - p[i]) * k / n for i in range(3))
            ps.pose.orientation = rot
            r.pose_stamped = ps
            res = self.call(self.ik, req)
            if res.error_code.val != 1:
                raise Abort(f'No collision-free IK {k} cm along the path (MoveIt code {res.error_code.val})')
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            q = [sol[nm] for nm in names]
            step = max(abs(wrap(a - b)) for a, b in zip(q, prev))
            if step > 0.3:
                raise Abort(f'The path needs a {step:.2f} rad joint change at {k} cm: configuration flip')
            largest = max(largest, step)
            prev = q
            seed = self.open_fingers(res.solution.joint_state)
        return dict(path_cm=round(dist * 100, 1), ik_points=n, largest_step_rad=round(largest, 3),
                    end_joints=[round(v, 3) for v in prev])

    def watch_line(self, a, b, q0, cmd_speed, prev_q):
        now = time.monotonic()
        if now - self.joint_time > STALE_SEC:
            raise Abort('Joint states are stale')
        p, rot, age = self.hand()
        if age > STALE_SEC:
            raise Abort(f'Hand TF is {age:.2f} s old')
        ab = [b[i] - a[i] for i in range(3)]
        L = math.sqrt(sum(v * v for v in ab)) or 1e-9
        s = sum((p[i] - a[i]) * ab[i] for i in range(3)) / L
        off = math.dist(p, [a[i] + ab[i] * max(0., min(1., s / L)) for i in range(3)])
        if off > LINE_MARGIN_M:
            raise Abort(f'Hand is {off*100:.1f} cm off the planned line')
        if s > L + LINE_MARGIN_M:
            raise Abort(f'Hand is {(s-L)*100:.1f} cm past the target')
        self.recent = [(t, v) for t, v in self.recent if now - t <= 0.2] + [(now, p)]
        span = now - self.recent[0][0]
        if span >= 0.1:
            speed = math.dist(p, self.recent[0][1]) / span
            if speed > SPEED_FACTOR * max(cmd_speed, SPEED_MPS) + SPEED_SLACK_MPS:
                raise Abort(f'Hand speed {speed*100:.1f} cm/s far above the command')
        turn = turn_deg(q0, rot)
        if turn > TURN_LIMIT_DEG:
            raise Abort(f'The tool turned {turn:.1f} deg under translation-only commands')
        q = self.joints()
        if prev_q is not None and max(abs(wrap(x - y)) for x, y in zip(q, prev_q)) > JUMP_RAD:
            raise Abort('A joint reading jumped (stale or corrupted joint feedback)')
        cmd = self.guard_cmd
        if any(abs(v) > 1e-9 for v in cmd[3:]):
            raise Abort(f'Guard forwarded an angular command {cmd}')
        if self.has_force:
            if now - self.force_time > STALE_SEC:
                raise Abort('Wrist force is stale')
            df = math.dist(self.mean_force(), self.start_force)
            if df > FORCE_CONTACT_N:
                raise Abort(f'Wrist force (0.25 s mean) changed by {df:.1f} N, above the rated payload: contact or snag')
        self.samples.append([round(now - self.t0, 4), *cmd[:3], *p, round(turn, 3), *q,
                             *(self.force or (float('nan'),) * 3)])
        return p, q, turn

    def move(self, a, target, q0):
        period = 1.0 / RATE_HZ
        begin = waiting_note = time.monotonic()
        prev_q = None
        while True:
            tick = time.monotonic()
            p, prev_q, turn = self.watch_line(a, target, q0, SPEED_MPS, prev_q)
            e = [target[i] - p[i] for i in range(3)]
            d = math.sqrt(sum(v * v for v in e))
            measured = math.dist(p, self.recent[0][1]) / max(1e-3, tick - self.recent[0][0]) if self.recent else 0.
            if d < self.args.tol + LAG_SEC * measured:
                break
            speed = max(MIN_SPEED_MPS, min(SPEED_MPS, d))
            self.send([e[i] / d * speed for i in range(3)] + [0.0] * 3)
            if any(abs(v) > 0 for v in self.guard_cmd[:3]):
                begin = tick
            elif tick - waiting_note > 5.0:
                print(f'    waiting for the deadman ({d*100:.1f} cm to go)', flush=True)
                waiting_note = tick
            if tick - begin > WAIT_SEC:
                raise Abort(f'No deadman for {WAIT_SEC:.0f} s')
            self.spin_for(max(0.0, period - (time.monotonic() - tick)))
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            self.send((0.0,) * 6)
            self.spin_for(period)
            p, prev_q, turn = self.watch_line(a, target, q0, SPEED_MPS, prev_q)
        return p, turn

    def run(self):
        self.t0 = time.monotonic()
        self.wait_for_inputs()
        target, label = self.target_in_arm_base()
        start, q0, _ = self.hand()
        report = dict(robot=self.args.robot, target=label, target_arm_base=[round(v, 4) for v in target],
                      start_arm_base=[round(v, 4) for v in start])
        report['preflight'] = self.preflight_path(target)
        print(f'Preflight passed: {json.dumps(report)}', flush=True)
        if self.args.check_only:
            return report
        self.publisher = self.create_publisher(PoseVelocity, self.command_topic, 10)
        self.start_force = self.mean_force() if self.has_force else None
        report['wrist_force_check'] = self.has_force
        try:
            print(f'Driving {math.dist(start, target)*100:.1f} cm at {SPEED_MPS*100:.0f} cm/s: '
                  'Fabric Follow (CBF) on for this robot, release and press the deadman, then hold it', flush=True)
            p, turn = self.move(start, target, q0)
            for _ in range(CORRECTIONS):
                if math.dist(p, target) <= 2 * self.args.tol:
                    break
                print(f'    {math.dist(p, target)*100:.1f} cm off: correcting', flush=True)
                p, turn = self.move(p, target, q0)
        finally:
            for _ in range(int(RATE_HZ)):
                self.send((0.0,) * 6)
                self.spin_for(1.0 / RATE_HZ)
        report['final_error_m'] = round(math.dist(p, target), 4)
        report['final_arm_base'] = [round(v, 4) for v in p]
        report['tool_turn_deg'] = round(turn, 2)
        report['final_joints'] = [round(v, 4) for v in self.joints()]
        print(f'Done: {report["final_error_m"]*100:.1f} cm from the target, tool turned {turn:.1f} deg; '
              f'joints {report["final_joints"]}', flush=True)
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robot', choices=('blue', 'silver'), required=True)
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument('--to', type=float, nargs=3, metavar=('X', 'Y', 'Z'), help='tool position in the world (m)')
    where.add_argument('--home', action='store_true', help='back to the tool position at the SRDF Home joints')
    parser.add_argument('--tol', type=float, default=TOL_M, help='stop within this distance of the target (m)')
    parser.add_argument('--check-only', action='store_true', help='run the preflight checks and send nothing')
    args = parser.parse_args()
    if args.to and not (-1.5 < args.to[0] < 1.5 and -2.0 < args.to[1] < 1.0 and 0.6 < args.to[2] < 1.5):
        parser.error('--to is outside the cell (x -1.5..1.5, y -2..1, z 0.6..1.5 m)')
    out = Path.home() / 'fabric_ws' / 'run' / 'grasp_drive'
    out.mkdir(parents=True, exist_ok=True)
    stem = out / f'{args.robot}_{datetime.now():%Y%m%d_%H%M%S}'
    rclpy.init()
    node = GraspDrive(args)
    report, code = {}, 0
    try:
        report = node.run()
    except Abort as e:
        report['abort'] = str(e)
        print(f'ABORT: {e}', flush=True)
        code = 1
    except KeyboardInterrupt:
        report['abort'] = 'interrupted'
        code = 130
    finally:
        if node.samples:
            with open(f'{stem}_samples.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'cmd_x', 'cmd_y', 'cmd_z', 'hand_x', 'hand_y', 'hand_z', 'turn_deg',
                            *[f'q{i}' for i in range(1, 7)], 'fx', 'fy', 'fz'])
                w.writerows(node.samples)
        Path(f'{stem}.json').write_text(json.dumps(report, indent=1))
        print(f'Saved {stem}.json', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
