#!/usr/bin/env python3
"""Response of one arm to the fabric bridge in joint-velocity mode, at its current pose (the grasp).

Kinova refuses Cartesian velocity at the fabric grasp pose (CartesianFaultState 1, 2026-11-18), so the
bridge converts the twin's world-frame grip velocity into joint rates (`output: joint`). This script
qualifies that path end to end: it starts the real `fabric_cbf_bridge` node in joint mode with a
workspace box of +/- --box metres around the grip's current position, publishes the cell calibration's
world -> base_link and end effector -> fabric_grip transforms, arms the bridge, and sends world-frame
legs of --step metres along each world axis and back at each speed through the bridge's normal input
(`fabric_cbf/cmd_world`). The command guard forwards them only in Fabric Follow (CBF) with the deadman
held: releasing it stops the arm; holding it again (release, press) resumes the leg.

Run on the robot's helper inside a fabric session (stack started, no fabric-cell or fabric-bridge):
    scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/joint_response.py --robot blue --check-only
    scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/joint_response.py --robot blue --axes z --speeds 0.02
Per leg it reports the grip's travel and direction in the world, the second-half speed, the lag from the
first forwarded joint command to motion, and the tool's rotation. Aborts only on faults: the grip more
than 3 cm beyond the planned travel or off the axis, the tool turning more than 3 deg, stale data, or the
bridge stopping (its diagnostics say why). Results: ~/fabric_ws/run/joint_response/.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped, TwistStamped
from kinova_msgs.msg import JointVelocity
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

REPO = Path(__file__).resolve().parents[2]
AXES = {'x': 0, 'y': 1, 'z': 2}
RATE_HZ = 50.0
STALE_SEC = 0.5
OFF_AXIS_M = 0.03
TURN_LIMIT_DEG = 3.0
SETTLE_SEC = 1.5


class Abort(RuntimeError):
    pass


def quat_matrix(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def turn_deg(Ra, Rb):
    return math.degrees(math.acos(max(-1., min(1., (np.trace(Ra.T @ Rb) - 1) / 2))))


class JointResponse(Node):
    def __init__(self, args):
        super().__init__(f'joint_response_{args.robot}')
        self.args, self.robot = args, args.robot
        self.ns, self.prefix = f'/oarbot_{args.robot}', f'oarbot_{args.robot}/'
        self.tool = self.prefix + 'fabric_grip'
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.static = tf2_ros.StaticTransformBroadcaster(self)
        self.joint_time = self.forward_time = None
        self.forwarded = np.zeros(6)
        self.bridge_status = None
        self.samples = []
        self.create_subscription(JointState, self.ns + '/joint_states', self.on_joints, 10)
        self.create_subscription(JointVelocity, self.ns + '/kinova/j2n6s300_driver/in/joint_velocity', self.on_forward, 10)
        self.create_subscription(DiagnosticArray, self.ns + '/diagnostics', self.on_diag, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, self.ns + '/fabric_cbf/cmd_world', 10)
        self.arm_pub = self.create_publisher(Bool, self.ns + '/fabric_cbf/arm', 1)

    def on_joints(self, msg):
        if any(n.startswith(self.prefix + 'j2n6s300_joint_') for n in msg.name):
            self.joint_time = time.monotonic()

    def on_forward(self, msg):
        self.forwarded = np.array([getattr(msg, f'joint{i}') for i in range(1, 7)])
        self.forward_time = time.monotonic()

    def on_diag(self, msg):
        for s in msg.status:
            if s.name.endswith('fabric_cbf_bridge'):
                self.bridge_status = (s.message, time.monotonic())

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def publish_calibration(self):
        cal = yaml.safe_load((REPO / 'experiments' / 'hardware' / 'cell' / 'calibration.yaml').read_text())
        out = []
        for t in cal['transforms']:
            if t['child'] in (self.prefix + 'base_link', self.tool):
                msg = TransformStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id, msg.child_frame_id = t['parent'], t['child']
                msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = t['translation']
                q = msg.transform.rotation
                q.x, q.y, q.z, q.w = t['rotation_xyzw']
                out.append(msg)
        if len(out) != 2:
            raise Abort('calibration.yaml lacks this robot\'s base_link or fabric_grip transform')
        self.static.sendTransform(out)
        return cal.get('measured', False)

    def grip(self):
        t = self.tf.lookup_transform('world', self.tool, Time())
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        p, q = t.transform.translation, t.transform.rotation
        return np.array([p.x, p.y, p.z]), quat_matrix(q.x, q.y, q.z, q.w), age

    def send(self, v):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(c) for c in v)
        self.cmd_pub.publish(msg)

    def forwarding(self):
        return self.forward_time is not None and time.monotonic() - self.forward_time < 0.1 and np.any(np.abs(self.forwarded) > 1e-6)

    def watch(self, start, R0, axis, sign, planned):
        now = time.monotonic()
        if self.joint_time is None or now - self.joint_time > STALE_SEC:
            raise Abort('Joint states are stale')
        p, R, age = self.grip()
        if age > STALE_SEC:
            raise Abort(f'Grip TF is {age:.2f} s old')
        d = p - start
        along = d[axis] * sign
        off = np.linalg.norm(np.delete(d, axis))
        if along > planned + OFF_AXIS_M or off > OFF_AXIS_M:
            raise Abort(f'Grip moved {100*along:.1f} cm along and {100*off:.1f} cm off the axis')
        turn = turn_deg(R0, R)
        if turn > TURN_LIMIT_DEG:
            raise Abort(f'The tool turned {turn:.1f} deg')
        if self.bridge_status and self.bridge_status[0] != 'Following the fabric twin' and now - self.bridge_status[1] < 1.0:
            raise Abort(f'Bridge stopped: {self.bridge_status[0]}')
        self.samples.append([round(now - self.t0, 4), axis, sign, *np.round(p, 5), round(turn, 3), *np.round(self.forwarded, 3)])
        return p, turn

    def leg(self, axis_name, sign, speed, origin, R0):
        axis = AXES[axis_name]
        v = np.zeros(3); v[axis] = sign * speed
        start, _, _ = self.grip()
        planned = self.args.step
        print(f'  leg {"+" if sign > 0 else "-"}{axis_name} at {100*speed:.1f} cm/s: hold the deadman', flush=True)
        period = 1.0 / RATE_HZ
        forwarded_s = 0.0
        first_cmd = first_move = None
        track = []
        begin = waiting = time.monotonic()
        while forwarded_s * speed < planned:
            tick = time.monotonic()
            self.send(v)
            rclpy.spin_once(self, timeout_sec=0.0)
            p, turn = self.watch(start, R0, axis, sign, planned)
            if self.forwarding():
                forwarded_s += period
                begin = tick
                first_cmd = first_cmd or tick
                track.append((forwarded_s * speed / planned, tick, p))
            elif tick - waiting > 5.0:
                print('    waiting: the guard is not forwarding (deadman released, or Fabric Follow off)', flush=True)
                waiting = tick
            if first_cmd and first_move is None and np.linalg.norm(p - start) > 0.002:
                first_move = tick
            if tick - begin > self.args.wait:
                raise Abort(f'No deadman for {self.args.wait:.0f} s')
            self.spin_for(max(0.0, period - (time.monotonic() - tick)))
        end = time.monotonic() + SETTLE_SEC
        while time.monotonic() < end:
            self.send(np.zeros(3))
            self.spin_for(period)
            p, turn = self.watch(start, R0, axis, sign, planned)
        disp = p - start
        moved = float(np.linalg.norm(disp))
        half = [(t, q) for f, t, q in track if f >= 0.5]
        steady = float(np.linalg.norm(half[-1][1] - half[0][1]) / (half[-1][0] - half[0][0])) if len(half) > 10 else None
        res = dict(axis=axis_name, sign=sign, speed_cmd_mps=speed, planned_m=planned, moved_m=round(moved, 4),
                   direction_world=np.round(disp / moved, 3).tolist() if moved > 1e-4 else None,
                   steady_speed_mps=None if steady is None else round(steady, 4),
                   lag_s=None if not (first_cmd and first_move) else round(first_move - first_cmd, 3),
                   tool_turn_deg=round(turn, 3))
        print(f'    moved {100*moved:.2f} of {100*planned:.1f} cm, direction {res["direction_world"]}, second-half speed '
              f'{"-" if steady is None else f"{100*steady:.2f} cm/s"}, lag {res["lag_s"]} s, tool turn {turn:.2f} deg', flush=True)
        return res


def bridge_command(args, box_min, box_max, cell):
    bridge = yaml.safe_load((cell / 'bridge.yaml').read_text())
    params = {**bridge['common'], **bridge.get(args.robot, {})}
    params.update(output='joint', workspace_min=[float(v) for v in box_min], workspace_max=[float(v) for v in box_max],
                  max_joint_dps=float(args.max_joint_dps))
    cmd = ['ros2', 'run', 'oarbot_runtime', 'fabric_cbf_bridge', '--ros-args', '-r', f'__ns:=/oarbot_{args.robot}']
    for k, v in params.items():
        if isinstance(v, bool):
            v = str(v).lower()
        cmd += ['-p', f'{k}:={json.dumps(v) if isinstance(v, (list, str)) else v}']
    return cmd


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', choices=('blue', 'silver'), required=True)
    ap.add_argument('--axes', default='zxy', help='world axes in order')
    ap.add_argument('--step', type=float, default=0.02, help='metres per leg (at most 0.03)')
    ap.add_argument('--speeds', default='0.01,0.02,0.03', help='m/s, each at most 0.05')
    ap.add_argument('--box', type=float, default=0.04, help='workspace box half-size around the grip (m)')
    ap.add_argument('--max-joint-dps', type=float, default=10.0)
    ap.add_argument('--wait', type=float, default=600.0, help='seconds to wait for the deadman')
    ap.add_argument('--check-only', action='store_true', help='start and arm the bridge, check, send nothing')
    args = ap.parse_args()
    args.speeds = [float(s) for s in args.speeds.split(',')]
    if not 0 < args.step <= 0.03 or not all(0 < s <= 0.05 for s in args.speeds) or set(args.axes) - set(AXES):
        ap.error('step at most 0.03 m, speeds in (0, 0.05] m/s, axes from xyz')
    if args.box < args.step + 0.01:
        ap.error('--box must exceed --step by at least 1 cm')
    out = Path.home() / 'fabric_ws' / 'run' / 'joint_response'
    out.mkdir(parents=True, exist_ok=True)
    stem = out / f'{args.robot}_{datetime.now():%Y%m%d_%H%M%S}'
    rclpy.init()
    node = JointResponse(args)
    report, code, bridge = dict(robot=args.robot, step_m=args.step, speeds_mps=args.speeds, axes=args.axes), 0, None
    try:
        node.t0 = time.monotonic()
        report['calibration_measured'] = node.publish_calibration()
        for topic in ('/kinova/j2n6s300_driver/in/cartesian_velocity', '/kinova/j2n6s300_driver/in/joint_velocity'):
            pubs = sorted(p.node_name for p in node.get_publishers_info_by_topic(node.ns + topic))
            if not set(pubs) <= {'command_guard', 'kinova_arm'} or 'command_guard' not in pubs:
                raise Abort(f'{node.ns + topic}: unexpected publishers {pubs}')
        if node.count_publishers(node.ns + '/commands/joint/arm') or node.count_subscribers(node.ns + '/fabric_cbf/cmd_world'):
            raise Abort('Another bridge or joint-command source is running (fabric-bridge or fabric-cell?)')
        end = time.monotonic() + 10
        while True:
            node.spin_for(0.2)
            try:
                p0, R0, age = node.grip()
                if node.joint_time is not None and age < STALE_SEC:
                    break
            except tf2_ros.TransformException:
                pass
            if time.monotonic() > end:
                raise Abort('No joint states or grip TF within 10 s')
        box_min, box_max = p0 - args.box, p0 + args.box
        report['grip_start_world'] = np.round(p0, 4).tolist()
        report['workspace_box'] = [np.round(box_min, 4).tolist(), np.round(box_max, 4).tolist()]
        cmd = bridge_command(args, box_min, box_max, REPO / 'experiments' / 'hardware' / 'cell')
        bridge = subprocess.Popen(cmd, stdout=open(f'{stem}_bridge.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
        print(f'Bridge (joint mode) started, box +/-{100*args.box:.0f} cm around the grip at {np.round(p0, 3).tolist()}', flush=True)
        end = time.monotonic() + 20
        while node.bridge_status is None or node.count_subscribers(node.ns + '/fabric_cbf/cmd_world') == 0:
            node.spin_for(0.2)
            if bridge.poll() is not None:
                raise Abort(f'The bridge exited (see {stem}_bridge.log)')
            if time.monotonic() > end:
                raise Abort('The bridge did not come up within 20 s')
        node.spin_for(1.0)
        node.arm_pub.publish(Bool(data=True))
        node.spin_for(1.0)
        status = node.bridge_status[0] if node.bridge_status else None
        report['bridge_after_arming'] = status
        print(f'Bridge after arming: {status}', flush=True)
        if status != 'Following the fabric twin':
            raise Abort(f'Bridge did not arm: {status}')
        if args.check_only:
            return 0
        print('Fabric Follow (CBF) on for this robot; release and press the deadman, then hold it', flush=True)
        report['legs'] = []
        for speed in args.speeds:
            for axis_name in args.axes:
                for sign in (1.0, -1.0):
                    report['legs'].append(node.leg(axis_name, sign, speed, p0, R0))
        p1, R1, _ = node.grip()
        report['return_error_m'] = round(float(np.linalg.norm(p1 - p0)), 4)
        report['tool_turn_end_deg'] = round(turn_deg(R0, R1), 3)
        print(f'Done: back within {100*report["return_error_m"]:.2f} cm of the start, tool turned {report["tool_turn_end_deg"]:.2f} deg', flush=True)
    except Abort as e:
        report['abort'] = str(e)
        print(f'ABORT: {e}', flush=True)
        code = 1
    except KeyboardInterrupt:
        report['abort'] = 'interrupted'
        code = 130
    finally:
        try:
            for _ in range(int(RATE_HZ)):
                node.send(np.zeros(3))
                node.spin_for(1.0 / RATE_HZ)
            node.arm_pub.publish(Bool(data=False))
            node.spin_for(0.3)
        except Exception:
            pass
        if bridge is not None and bridge.poll() is None:
            os.killpg(bridge.pid, signal.SIGINT)
            try:
                bridge.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(bridge.pid, signal.SIGKILL)
        if node.samples:
            np.savetxt(f'{stem}_samples.csv', np.array(node.samples), delimiter=',',
                       header='t,axis,sign,x,y,z,turn_deg,qd1,qd2,qd3,qd4,qd5,qd6', comments='')
        Path(f'{stem}.json').write_text(json.dumps(report, indent=1))
        print(f'Saved {stem}.json', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
