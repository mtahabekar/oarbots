#!/usr/bin/env python3
"""First supervised arm motion: gate H3, translation only.

Moves one arm along one Kinova command axis and back (default 5 cm), axis by axis, at each speed
of a short sweep (default 2, 3, 4 cm/s), with zero angular velocity. On 2026-11-16 Silver did not
move at all at 1 cm/s and only 1 mm in 0.75 s at 2 cm/s, so the sweep measures where the firmware
starts to follow. Commands go to the command guard's fabric_cbf input, so the arm moves only while
the operator holds the tablet deadman with "Fabric Follow (CBF)" selected for this arm. Releasing
the deadman stops the arm; the script counts only the distance the guard actually forwarded, so a
leg pauses and resumes with the deadman instead of overshooting.

Run on the robot's helper laptop inside a fabric session (stack started, fabric-cell not running):
    scripts/hw/lab.sh session silver exec python3 ~/fabric_ws/scripts/hw/first_motion.py --robot silver --check-only
    scripts/hw/lab.sh session silver exec python3 ~/fabric_ws/scripts/hw/first_motion.py --robot silver --axes z

Before any command it checks that the arm is at the SRDF Home state, that the guard is the only
publisher to the driver and nothing else publishes to the fabric_cbf input, and that MoveIt finds
every centimetre along the tested axes (both signs) and a 3 cm sphere around the hand collision-free
and reachable without a configuration change. The deadman is the primary stop. The script aborts
only on faults: the hand straying 3 cm beyond the planned travel, a joint moving more than the
IK-predicted change allows, hand speed far above the command (Kinova velocity units unverified),
stale data, motion forwarded on another axis, or a filtered wrist-force change above the arm's
rated payload (contact or snag).
"""
import argparse
import csv
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped, WrenchStamped
from kinova_msgs.msg import PoseVelocity
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
import tf2_ros

FIELDS = ('twist_linear_x', 'twist_linear_y', 'twist_linear_z', 'twist_angular_x', 'twist_angular_y', 'twist_angular_z')
AXES = {'x': 0, 'y': 1, 'z': 2}
RATE_HZ = 50.0
STALE_SEC = 0.5          # joint states arrive at 50 Hz and TF follows them; 0.5 s is 25 missed updates
HOME_TOL_RAD = 0.05      # measured offsets from Home were 0.001-0.014 rad on 2026-11-16
FORCE_CONTACT_N = 15.7   # Jaco2 rated payload 1.6 kg at mid-range: a sustained change above it is a snag/contact
FORCE_WINDOW_SEC = 0.25  # raw force swings +/-4 N when the arm jerks into motion (measured); average it out
TRAVEL_MARGIN_M = 0.03   # abort if the hand strays this far beyond the planned travel
SETTLE_SEC = 1.5
SPEED_FACTOR = 3.0       # Kinova velocity units/scaling are unverified on this firmware (platform audit H3):
SPEED_SLACK_MPS = 0.01   # abort if the hand moves faster than 3x the command plus measurement noise
LEG_TIMEOUT_SEC = 120.0  # how long a leg waits for the operator's deadman before giving up


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def srdf_home(robot):
    path = Path(get_package_share_directory(f'oarbot_{robot}_moveit')) / 'config' / f'oarbot_{robot}.srdf'
    for state in ET.parse(path).getroot().iter('group_state'):
        if state.get('name') == 'Home' and state.get('group') == 'arm':
            return {j.get('name'): float(j.get('value')) for j in state.iter('joint')}
    raise RuntimeError(f'No arm Home state in {path}')


def directions():
    """Six axis directions and eight diagonals: the neighbourhood MoveIt must clear."""
    out = [tuple(s * (i == k) for k in range(3)) for i in range(3) for s in (1.0, -1.0)]
    d = 1 / math.sqrt(3)
    out += [(sx * d, sy * d, sz * d) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    return out


class Abort(RuntimeError):
    pass


class FirstMotion(Node):
    def __init__(self, args):
        super().__init__(f'first_motion_{args.robot}')
        self.args = args
        self.ns = f'/oarbot_{args.robot}'
        prefix = f'oarbot_{args.robot}/'
        self.base, self.ee, self.robot_base = prefix + 'j2n6s300_link_base', prefix + 'j2n6s300_end_effector', prefix + 'base_link'
        self.arm_joints = [prefix + f'j2n6s300_joint_{i}' for i in range(1, 7)]
        self.guard_topic = self.ns + '/kinova/j2n6s300_driver/in/cartesian_velocity'
        self.command_topic = self.ns + '/commands/fabric_cbf/arm'
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.joint_msg = None
        self.joint_time = self.force_time = self.guard_time = None
        self.force = self.start_force = None
        self.force_window = []   # (time, force) over the last FORCE_WINDOW_SEC
        self.leg_speed = 0.0
        self.recent = []   # (time, hand position) over the last ~0.2 s, for the speed check
        self.guard_cmd = (0.0,) * 6
        self.forwarded = [0.0, 0.0, 0.0]   # distance the guard forwarded along each command axis
        self.samples = []
        self.create_subscription(JointState, self.ns + '/joint_states', self.on_joints, 10)
        self.create_subscription(PoseVelocity, self.guard_topic, self.on_guard, 10)
        self.create_subscription(WrenchStamped, self.ns + '/force_torque_raw', self.on_force, qos_profile_sensor_data)
        self.ik = self.create_client(GetPositionIK, self.ns + '/compute_ik')
        self.validity = self.create_client(GetStateValidity, self.ns + '/check_state_validity')
        self.publisher = None

    # ----- inputs
    def on_joints(self, msg):
        if all(name in msg.name for name in self.arm_joints):
            self.joint_msg, self.joint_time = msg, time.monotonic()

    def on_force(self, msg):
        f = msg.wrench.force
        now = time.monotonic()
        self.force, self.force_time = (f.x, f.y, f.z), now
        self.force_window = [(t, v) for t, v in self.force_window if now - t <= FORCE_WINDOW_SEC] + [(now, self.force)]

    def mean_force(self):
        n = len(self.force_window)
        return tuple(sum(v[i] for _, v in self.force_window) / n for i in range(3))

    def on_guard(self, msg):
        now = time.monotonic()
        if self.guard_time is not None:
            dt = min(now - self.guard_time, 0.05)
            for i in range(3):
                self.forwarded[i] += self.guard_cmd[i] * dt
        self.guard_cmd, self.guard_time = tuple(getattr(msg, f) for f in FIELDS), now

    # ----- helpers
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def joints(self):
        pos = dict(zip(self.joint_msg.name, self.joint_msg.position))
        return [pos[n] for n in self.arm_joints]

    def hand(self, frame=None):
        t = self.tf.lookup_transform(frame or self.base, self.ee, Time())
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        p = t.transform.translation
        return (p.x, p.y, p.z), t.transform.rotation, age

    def rotate_to_robot_base(self, vector):
        q = self.tf.lookup_transform(self.robot_base, self.base, Time()).transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = [[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
               [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
               [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]]
        return tuple(sum(rot[r][c] * vector[c] for c in range(3)) for r in range(3))

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise Abort(f'{client.srv_name} is not available; is the fabric session stack running?')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise Abort(f'{client.srv_name} did not answer')
        return future.result()

    def send(self, values):
        self.publisher.publish(PoseVelocity(**dict(zip(FIELDS, values))))

    # ----- checks
    def wait_for_inputs(self):
        self.spin_for(2.0)   # let discovery finish before counting publishers
        need_force = self.count_publishers(self.ns + '/force_torque_raw') > 0
        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                self.hand()
            except tf2_ros.TransformException:
                continue
            if self.joint_msg is not None and self.guard_time is not None and (self.force is not None or not need_force):
                self.has_force = need_force
                return
        raise Abort('No joint states, TF, guard output or wrist force within 10 s')

    def preflight(self):
        home = srdf_home(self.args.robot)
        q = self.joints()
        home_err = max(abs(wrap(qi - home[n])) for qi, n in zip(q, self.arm_joints))
        if home_err > HOME_TOL_RAD:
            raise Abort(f'Arm is not at Home: largest joint offset {home_err:.3f} rad > {HOME_TOL_RAD}')

        guard_pubs = self.get_publishers_info_by_topic(self.guard_topic)
        if [p.node_name for p in guard_pubs] != ['command_guard']:
            raise Abort(f'Driver input must have only command_guard as publisher, found {[p.node_name for p in guard_pubs]}')
        if self.count_publishers(self.command_topic):
            raise Abort(f'{self.command_topic} already has a publisher (is fabric-cell running?)')

        req = GetStateValidity.Request()
        req.robot_state = RobotState(joint_state=self.joint_msg)
        req.group_name = 'arm'
        res = self.call(self.validity, req)
        if not res.valid:
            raise Abort(f'MoveIt reports the current state invalid: {[(c.contact_body_1, c.contact_body_2) for c in res.contacts]}')

        p, rot, _ = self.hand()
        targets = [(d, r) for d in directions() for r in (0.015, 0.03)]
        travel = self.args.step + 0.01
        for axis in set(self.args.axes):
            unit = tuple(float(i == AXES[axis]) for i in range(3))
            for sign in (1.0, -1.0):
                targets += [(tuple(sign * u for u in unit), 0.01 * k) for k in range(1, int(round(travel * 100)) + 1)]
        largest = 0.0
        for d, dist in targets:
            for scale in (1.0,):
                req = GetPositionIK.Request()
                req.ik_request.group_name = 'arm'
                req.ik_request.robot_state = RobotState(joint_state=self.joint_msg)
                req.ik_request.avoid_collisions = True
                req.ik_request.ik_link_name = self.ee
                req.ik_request.timeout = DurationMsg(sec=0, nanosec=200_000_000)
                target = PoseStamped()
                target.header.frame_id = self.base
                target.pose.position.x, target.pose.position.y, target.pose.position.z = (
                    p[i] + dist * d[i] for i in range(3))
                target.pose.orientation = rot
                req.ik_request.pose_stamped = target
                res = self.call(self.ik, req)
                if res.error_code.val != 1:
                    raise Abort(f'No collision-free IK {dist*100:.1f} cm along {d} (MoveIt code {res.error_code.val})')
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                change = max(abs(wrap(sol[n] - qi)) for n, qi in zip(self.arm_joints, q))
                if change > 0.3 + 10 * dist:
                    raise Abort(f'IK {dist*100:.1f} cm along {d} needs a {change:.2f} rad joint change: configuration flip')
                largest = max(largest, change)
        # Joints may move at most what reaching the whole checked neighbourhood needs, plus margin.
        self.joint_bound = max(0.05, 1.5 * largest)
        return dict(home_offset_rad=home_err, ik_targets_checked=len(targets), ik_max_joint_change_rad=largest,
                    joint_bound_rad=self.joint_bound, axis_travel_checked_cm=travel * 100, hand_in_arm_base_m=p)

    def watch(self, start_hand, start_joints, axis):
        now = time.monotonic()
        if now - self.joint_time > STALE_SEC:
            raise Abort('Joint states are stale')
        p, _, age = self.hand()
        if age > STALE_SEC:
            raise Abort(f'Hand TF is {age:.2f} s old')
        moved = math.dist(p, start_hand)
        self.recent = [(t, q) for t, q in self.recent if now - t <= 0.2] + [(now, p)]
        span = now - self.recent[0][0]
        if span >= 0.1:
            speed = math.dist(p, self.recent[0][1]) / span
            if speed > SPEED_FACTOR * self.leg_speed + SPEED_SLACK_MPS:
                raise Abort(f'Hand speed {speed*100:.1f} cm/s exceeds {SPEED_FACTOR:.0f}x the {self.leg_speed*100:.1f} cm/s command')
        if moved > self.args.step + TRAVEL_MARGIN_M:
            raise Abort(f'Hand moved {moved*100:.1f} cm from the start, {TRAVEL_MARGIN_M*100:.0f} cm beyond the planned travel')
        q = self.joints()
        jmax = max(abs(wrap(a - b)) for a, b in zip(q, start_joints))
        if jmax > self.joint_bound:
            raise Abort(f'A joint moved {jmax:.3f} rad, more than the {self.joint_bound:.3f} rad bound')
        cmd = self.guard_cmd
        if any(abs(v) > 1e-9 for v in cmd[3:]) or any(abs(cmd[i]) > 1e-9 for i in range(3) if i != axis):
            raise Abort(f'Guard forwarded an unexpected command {cmd}')
        if self.has_force:
            if now - self.force_time > STALE_SEC:
                raise Abort('Wrist force is stale')
            df = math.dist(self.mean_force(), self.start_force)
            if df > FORCE_CONTACT_N:
                raise Abort(f'Wrist force (0.25 s mean) changed by {df:.1f} N, above the rated payload: contact or snag')
        self.samples.append([round(now - self.t0, 4), axis, *cmd[:3], *self.forwarded, *p, *q,
                             *(self.force or (float('nan'),) * 3)])
        return p

    # ----- motion
    def leg(self, axis_name, sign, speed, start_hand, start_joints):
        axis = AXES[axis_name]
        target = self.args.step
        self.forwarded = [0.0, 0.0, 0.0]
        self.leg_speed = speed
        p0, _, _ = self.hand()
        first_cmd = first_move = None
        track = []   # (forwarded fraction, time, hand) for the steady-speed estimate
        command = [0.0] * 6
        command[axis] = sign * speed
        print(f'  leg {"+" if sign > 0 else "-"}{axis_name} at {speed*100:.1f} cm/s: hold the deadman', flush=True)
        begin = time.monotonic()
        waiting_note = begin
        period = 1.0 / RATE_HZ
        while abs(self.forwarded[axis]) < target:
            tick = time.monotonic()
            if tick - begin > LEG_TIMEOUT_SEC:
                raise Abort(f'No deadman for {LEG_TIMEOUT_SEC:.0f} s')
            self.send(command)
            rclpy.spin_once(self, timeout_sec=0.0)
            p = self.watch(start_hand, start_joints, axis)
            if first_cmd is None and abs(self.guard_cmd[axis]) > 0:
                first_cmd = tick
            if first_cmd is not None and first_move is None and math.dist(p, p0) > 0.002:
                first_move = tick
            if abs(self.guard_cmd[axis]) > 0:
                track.append((abs(self.forwarded[axis]) / target, tick, p))
            if abs(self.guard_cmd[axis]) == 0 and tick - waiting_note > 5.0:
                print('    waiting: guard is not forwarding (deadman released, or Fabric Follow not selected)', flush=True)
                waiting_note = tick
            self.spin_for(max(0.0, period - (time.monotonic() - tick)))
        end = time.monotonic() + SETTLE_SEC
        while time.monotonic() < end:
            self.send((0.0,) * 6)
            self.spin_for(period)
            self.watch(start_hand, start_joints, axis)
        p1, _, _ = self.hand()
        disp = tuple(b - a for a, b in zip(p0, p1))
        norm = math.sqrt(sum(v * v for v in disp))
        half = [(t, q) for frac, t, q in track if frac >= 0.5]
        steady = math.dist(half[0][1], half[-1][1]) / (half[-1][0] - half[0][0]) if len(half) > 10 else None
        result = dict(axis=axis_name, sign=sign, speed_cmd_mps=speed, forwarded_m=self.forwarded[axis], moved_m=norm,
                      steady_speed_mps=steady,
                      direction_arm_base=[round(v / norm, 3) for v in disp] if norm > 1e-4 else None,
                      direction_robot_base=[round(v / norm, 3) for v in self.rotate_to_robot_base(disp)] if norm > 1e-4 else None,
                      lag_s=(first_move - first_cmd) if first_cmd and first_move else None)
        print(f'    forwarded {self.forwarded[axis]*100:.2f} cm, moved {norm*100:.2f} cm, second-half speed '
              f'{"-" if steady is None else f"{steady*100:.2f} cm/s"}, lag {"-" if result["lag_s"] is None else f"{result["lag_s"]:.2f} s"}, '
              f'direction in robot base {result["direction_robot_base"]}', flush=True)
        return result

    def run(self):
        self.t0 = time.monotonic()
        self.wait_for_inputs()
        report = dict(robot=self.args.robot, step_m=self.args.step, speeds_mps=self.args.speeds, axes=self.args.axes)
        report['preflight'] = self.preflight()
        print(f'Preflight passed: {json.dumps(report["preflight"])}', flush=True)
        if self.args.check_only:
            return report
        self.publisher = self.create_publisher(PoseVelocity, self.command_topic, 10)
        self.start_force = self.mean_force() if self.has_force else None
        report['wrist_force_check'] = self.has_force
        start_hand, _, _ = self.hand()
        start_joints = self.joints()
        report['legs'] = []
        try:
            for speed in self.args.speeds:
                for axis_name in self.args.axes:
                    for sign in (1.0, -1.0):
                        report['legs'].append(self.leg(axis_name, sign, speed, start_hand, start_joints))
            final, _, _ = self.hand()
            report['return_error_m'] = math.dist(final, start_hand)
        finally:
            for _ in range(int(RATE_HZ)):
                self.send((0.0,) * 6)
                self.spin_for(1.0 / RATE_HZ)
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robot', choices=('blue', 'silver'), required=True)
    parser.add_argument('--axes', default='z', help='command axes in order, e.g. z or zxy')
    parser.add_argument('--step', type=float, default=0.05, help='metres per leg (at most 0.10)')
    parser.add_argument('--speeds', default='0.02,0.03,0.04', help='comma-separated m/s, each at most 0.05')
    parser.add_argument('--check-only', action='store_true', help='run the preflight checks and send nothing')
    args = parser.parse_args()
    args.speeds = [float(v) for v in args.speeds.split(',') if v.strip()]
    if not (0 < args.step <= 0.10 and args.speeds and all(0 < v <= 0.05 for v in args.speeds)
            and args.axes and set(args.axes) <= set(AXES)):
        parser.error('step must be in (0, 0.10] m, speeds in (0, 0.05] m/s, axes from x, y, z')
    args.axes = list(args.axes)

    rclpy.init()
    node = FirstMotion(args)
    out_dir = Path.home() / 'fabric_ws' / 'run' / 'first_motion' / f'{args.robot}_{time.strftime("%Y%m%d_%H%M%S")}'
    report, code = {}, 0
    try:
        report = node.run()
    except Abort as error:
        report['aborted'] = str(error)
        print(f'ABORTED: {error}', flush=True)
        code = 2
    except KeyboardInterrupt:
        report['aborted'] = 'interrupted'
        code = 130
    finally:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / 'report.json').write_text(json.dumps(report, indent=2))
        with (out_dir / 'samples.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'axis', 'cmd_x', 'cmd_y', 'cmd_z', 'fwd_x', 'fwd_y', 'fwd_z', 'hand_x', 'hand_y', 'hand_z',
                             *[f'q{i}' for i in range(1, 7)], 'fx', 'fy', 'fz'])
            writer.writerows(node.samples)
        print(f'Report: {out_dir}/report.json', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
