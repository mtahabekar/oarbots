#!/usr/bin/env python3
"""Move one arm in joint space to the grasp pose (or back to Home) through the command guard.

A straight Cartesian move from Kinova Home to the grasp pose crosses a wrist singularity (joint 5 near
0; lab log 2026-11-18), so the approach is a straight line in joint space instead: every joint moves
in proportion, the largest at --speed deg/s. Commands go to the guard's joint input
(commands/joint/arm), which forwards them only in the fabric_cbf mode with the tablet deadman held:
releasing the deadman stops the arm, and the path resumes where it stopped when the deadman is held
again (release and press). After arriving the script keeps the arm still until the deadman is released,
so Silver's guard does not close its fingers before the corner is in them.

Run on the robot's helper inside a fabric session; both robots' sessions running (the other arm's
position is checked against the path):
    scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot blue --check-only
    scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot blue
    scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot blue --to home
Targets: `grasp` (experiments/hardware/cell/grasp.yaml), `home` (the SRDF Home state) or six joint
angles in rad. Preflight: MoveIt finds every degree of the joint path collision-free, and the tool,
wrist and camera stay at least 15 cm from the other arm. Aborts only on faults: stale or jumping joint
readings, a joint more than 10 deg off the path (the arm not following), joints faster than 3x the
command, the guard forwarding a command it was not given, or a gravity-compensated wrist-force change
above the rated payload.
"""
import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import WrenchStamped
from kinova_msgs.msg import JointVelocity
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parent))
from first_motion import Abort, srdf_home  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
CONTINUOUS = (0, 3, 4, 5)        # joints 1, 4, 5, 6 turn without limits; 2 and 3 have limits
RATE_HZ = 50.0
STALE_SEC = 0.5
TRACK_LIMIT_DEG = 10.0
JUMP_RAD = 0.1
SPEED_FACTOR = 3.0
GAIN_PER_S = 1.0
ACCEL_SEC = 1.0                  # ramp of the path speed at start and end
FORCE_CONTACT_N = 15.7
FORCE_WINDOW_SEC = 0.25
CLEARANCE_M = 0.15
WAIT_SEC = 120.0
LINKS = ('j2n6s300_end_effector', 'j2n6s300_link_5', 'camera_base')


def quat_matrix(x, y, z, w):
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def base_in_world(robot):
    cal = yaml.safe_load((REPO / 'experiments' / 'hardware' / 'cell' / 'calibration.yaml').read_text())
    for t in cal['transforms']:
        if t['parent'] == 'world' and t['child'] == f'oarbot_{robot}/base_link':
            T = np.eye(4)
            T[:3, :3] = quat_matrix(*t['rotation_xyzw'])
            T[:3, 3] = t['translation']
            return T
    raise Abort(f'No world -> oarbot_{robot}/base_link in calibration.yaml')


def delta(q0, q1):
    d = np.array(q1, float) - np.array(q0, float)
    for i in CONTINUOUS:
        d[i] = (d[i] + math.pi) % (2 * math.pi) - math.pi
    return d


class GraspJoints(Node):
    def __init__(self, args):
        super().__init__(f'grasp_joints_{args.robot}')
        self.args, self.robot = args, args.robot
        self.other = 'silver' if args.robot == 'blue' else 'blue'
        self.ns = f'/oarbot_{self.robot}'
        self.prefix = f'oarbot_{self.robot}/'
        self.names = [self.prefix + f'j2n6s300_joint_{i}' for i in range(1, 7)]
        self.joint_msg = self.joint_time = None
        self.forwarded = None            # last joint command the guard sent to the driver, and when
        self.forwarded_time = None
        self.deadman = None
        self.force_window = []
        self.samples = []
        self.create_subscription(JointState, self.ns + '/joint_states', self.on_joints, 10)
        self.driver_topic = self.ns + '/kinova/j2n6s300_driver/in/joint_velocity'
        self.create_subscription(JointVelocity, self.driver_topic, self.on_forwarded, 10)
        self.create_subscription(Bool, '/tablet_gui/deadman', lambda m: setattr(self, 'deadman', m.data), 10)
        self.create_subscription(WrenchStamped, self.ns + '/force_torque_calibrated', self.on_force, qos_profile_sensor_data)
        self.command_topic = self.ns + '/commands/joint/arm'
        self.fk = self.create_client(GetPositionFK, self.ns + '/compute_fk')
        self.validity = self.create_client(GetStateValidity, self.ns + '/check_state_validity')
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.publisher = None

    def on_joints(self, msg):
        if all(n in msg.name for n in self.names):
            self.joint_msg, self.joint_time = msg, time.monotonic()

    def on_forwarded(self, msg):
        self.forwarded = [getattr(msg, f'joint{i}') for i in range(1, 7)]
        self.forwarded_time = time.monotonic()

    def on_force(self, msg):
        now = time.monotonic()
        f = msg.wrench.force
        self.force_window = [(t, v) for t, v in self.force_window if now - t <= FORCE_WINDOW_SEC] + [(now, (f.x, f.y, f.z))]

    def mean_force(self):
        if not self.force_window:
            return None
        return np.mean([v for _, v in self.force_window], axis=0)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def joints(self):
        pos = dict(zip(self.joint_msg.name, self.joint_msg.position))
        return np.array([pos[n] for n in self.names])

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise Abort(f'{client.srv_name} is not available; is the fabric session stack running?')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise Abort(f'{client.srv_name} did not answer')
        return future.result()

    def state(self, q):
        js = JointState()
        js.name = list(self.joint_msg.name)
        js.position = list(self.joint_msg.position)
        for n, v in zip(self.names, q):
            js.position[js.name.index(n)] = float(v)
        return RobotState(joint_state=js)

    def points(self, q, W):
        req = GetPositionFK.Request()
        req.header.frame_id = self.prefix + 'base_link'
        req.fk_link_names = [self.prefix + n for n in LINKS]
        req.robot_state = self.state(q)
        res = self.call(self.fk, req)
        if res.error_code.val != 1:
            raise Abort(f'compute_fk failed (MoveIt code {res.error_code.val})')
        return [(W @ [p.pose.position.x, p.pose.position.y, p.pose.position.z, 1.])[:3] for p in res.pose_stamped]

    def other_points(self):
        W = base_in_world(self.other)
        out = []
        for n in LINKS:
            frame = f'oarbot_{self.other}/{n}'
            end = time.monotonic() + 5.0
            while not self.tf.can_transform(f'oarbot_{self.other}/base_link', frame, Time()):
                rclpy.spin_once(self, timeout_sec=0.05)
                if time.monotonic() > end:
                    raise Abort(f'No TF for {frame}: start {self.other}\'s session so its arm can be checked against the path')
            t = self.tf.lookup_transform(f'oarbot_{self.other}/base_link', frame, Time()).transform.translation
            out.append((W @ [t.x, t.y, t.z, 1.])[:3])
        return out

    def target(self):
        if self.args.to == ['grasp']:
            q = yaml.safe_load((REPO / 'experiments' / 'hardware' / 'cell' / 'grasp.yaml').read_text())[self.robot]
            return np.array(q, float), 'grasp (grasp.yaml)'
        if self.args.to == ['home']:
            home = srdf_home(self.robot)
            return np.array([home[n] for n in self.names]), 'Home (SRDF)'
        if len(self.args.to) == 6:
            return np.array([float(v) for v in self.args.to]), 'joints given'
        raise Abort('--to is grasp, home or six joint angles')

    def preflight(self, q0, q1):
        cart = self.ns + '/kinova/j2n6s300_driver/in/cartesian_velocity'
        pubs = [p.node_name for p in self.get_publishers_info_by_topic(cart)]
        if pubs != ['command_guard']:
            raise Abort(f'{cart} must have only command_guard as publisher, found {pubs}')
        # The driver's own trajectory controller (node kinova_arm) also has a publisher on the joint input;
        # it sends only while executing a FollowJointTrajectory goal, which nothing in the session sends.
        # The guard sends joint commands only when given some: so the input must be silent now.
        pubs = sorted(p.node_name for p in self.get_publishers_info_by_topic(self.driver_topic))
        if pubs != ['command_guard', 'kinova_arm']:
            raise Abort(f'{self.driver_topic}: expected command_guard (with the joint input) and the driver\'s idle '
                        f'trajectory controller as publishers, found {pubs}')
        self.forwarded_time = None
        self.spin_for(1.0)
        if self.forwarded_time is not None:
            raise Abort(f'{self.driver_topic} is not silent before the move: another source is commanding joints')
        if self.count_publishers(self.command_topic):
            raise Abort(f'{self.command_topic} already has a publisher')
        d = delta(q0, q1)
        steps = max(1, int(math.ceil(np.degrees(np.max(np.abs(d))))))
        W = base_in_world(self.robot)
        others = self.other_points()
        closest = (math.inf, None)
        tool_path = []
        for k in range(steps + 1):
            q = q0 + d * k / steps
            req = GetStateValidity.Request()
            req.robot_state = self.state(q)
            req.group_name = 'arm'
            res = self.call(self.validity, req)
            if not res.valid:
                raise Abort(f'MoveIt: collision at {100*k/steps:.0f} % of the path: '
                            f'{sorted({(c.contact_body_1, c.contact_body_2) for c in res.contacts})}')
            if k % 5 == 0 or k == steps:
                pts = self.points(q, W)
                tool_path.append(pts[0])
                for a, name in zip(pts, LINKS):
                    for b in others:
                        dist = float(np.linalg.norm(a - b))
                        if dist < closest[0]:
                            closest = (dist, f'{name} at {100*k/steps:.0f} %')
        if closest[0] < CLEARANCE_M:
            raise Abort(f'The path comes {closest[0]*100:.0f} cm from {self.other}\'s arm ({closest[1]})')
        tool_path = np.array(tool_path)
        return dict(joint_change_deg=np.round(np.degrees(d), 1).tolist(), checks=steps + 1,
                    seconds_at_speed=round(float(np.degrees(np.max(np.abs(d)))) / self.args.speed + 2 * ACCEL_SEC, 1),
                    closest_to_other_arm_m=round(closest[0], 3), closest_where=closest[1],
                    tool_start_world=np.round(tool_path[0], 3).tolist(), tool_end_world=np.round(tool_path[-1], 3).tolist(),
                    tool_lowest_z=round(float(tool_path[:, 2].min()), 3),
                    tool_x_range=[round(float(tool_path[:, 0].min()), 3), round(float(tool_path[:, 0].max()), 3)])

    def send(self, v_deg):
        self.publisher.publish(JointVelocity(**{f'joint{i}': float(v) for i, v in enumerate(v_deg, 1)}))

    def drive(self, q0, q1):
        d = delta(q0, q1)
        span = float(np.degrees(np.max(np.abs(d))))
        rate = self.args.speed / span            # path fraction per second at full speed
        s, sdot = 0.0, 0.0
        period = 1.0 / RATE_HZ
        prev_q, prev_t = None, None
        f0 = self.mean_force()
        last_progress = waiting_note = time.monotonic()
        done = arrived_note = False
        finished_at = None
        recent = []                              # (time, largest commanded |v|) over the last 0.3 s
        while True:
            tick = time.monotonic()
            if tick - self.joint_time > STALE_SEC:
                raise Abort('Joint states are stale')
            q = self.joints()
            if prev_q is not None:
                step = np.abs(delta(prev_q, q))
                if np.max(step) > JUMP_RAD:
                    raise Abort(f'A joint reading jumped {np.max(step):.3f} rad (stale or corrupted feedback)')
                speed = np.degrees(np.max(step)) / max(1e-3, tick - prev_t)
                if speed > SPEED_FACTOR * self.args.speed + 3.0:
                    raise Abort(f'A joint moved at {speed:.0f} deg/s, far above the {self.args.speed:.0f} deg/s command')
            prev_q, prev_t = q, tick
            forwarding = (self.forwarded_time is not None and tick - self.forwarded_time < 0.1)
            # the path advances only while the guard forwards (deadman held); it ramps up and down
            remaining = 1.0 - s
            target_rate = rate if forwarding and not done else 0.0
            target_rate = min(target_rate, math.sqrt(2 * remaining * rate / ACCEL_SEC)) if remaining > 0 else 0.0
            sdot += max(-rate / ACCEL_SEC * period, min(rate / ACCEL_SEC * period, target_rate - sdot))
            sdot = max(0.0, sdot)
            if not forwarding:
                sdot = 0.0
            s = min(1.0, s + sdot * period)
            ref = q0 + d * s
            err = np.degrees(delta(q, ref))
            if np.max(np.abs(err)) > TRACK_LIMIT_DEG and forwarding:
                raise Abort(f'Joint {int(np.argmax(np.abs(err))) + 1} is {np.max(np.abs(err)):.1f} deg off the path: '
                            'the arm is not following')
            v = np.degrees(d) * sdot + GAIN_PER_S * err
            if s >= 1.0:
                finished_at = finished_at or tick
                # within 1 deg, or settled for 3 s (a residual below a joint's velocity deadband)
                if np.max(np.abs(np.degrees(delta(q, q1)))) < 1.0 or tick - finished_at > 3.0:
                    done = True
            if done:
                v = np.zeros(6)
            v = np.clip(v, -self.args.speed * 1.2, self.args.speed * 1.2)
            self.send(v)
            recent = [(t, m) for t, m in recent if tick - t <= 0.3] + [(tick, float(np.max(np.abs(v))))]
            if self.forwarded is not None and forwarding:
                if np.max(np.abs(self.forwarded)) > max(m for _, m in recent) + 2.0:
                    raise Abort(f'The guard forwarded {np.round(self.forwarded, 1).tolist()}, more than was commanded')
            f = self.mean_force()
            if f is not None and f0 is not None and np.linalg.norm(f - f0) > FORCE_CONTACT_N:
                raise Abort(f'Compensated wrist force (0.25 s mean) changed by {np.linalg.norm(f - f0):.1f} N: contact or snag')
            self.samples.append([round(tick - self.t0, 3), round(s, 4), int(forwarding), *np.round(v, 3), *np.round(q, 5),
                                 *(np.round(f, 2) if f is not None else [math.nan] * 3)])
            if forwarding and sdot > 0:
                last_progress = tick
            if done and (self.deadman is False or not forwarding):
                return q
            if not forwarding and tick - waiting_note > 5.0:
                print(f'    waiting for the deadman ({100*s:.0f} % of the path done)', flush=True)
                waiting_note = tick
            if tick - last_progress > WAIT_SEC and not done:
                raise Abort(f'No deadman for {WAIT_SEC:.0f} s')
            if done and tick - last_progress > 60.0:
                return q
            if done and not arrived_note:
                arrived_note = True
                print('Arrived. Release the deadman now.', flush=True)
            self.spin_for(max(0.0, period - (time.monotonic() - tick)))

    def run(self):
        self.t0 = time.monotonic()
        end = time.monotonic() + 10.0
        while self.joint_msg is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.joint_msg is None:
            raise Abort('No joint states within 10 s')
        self.spin_for(1.0)
        q0 = self.joints()
        q1, label = self.target()
        report = dict(robot=self.robot, target=label, start_joints=np.round(q0, 4).tolist(),
                      target_joints=np.round(q1, 4).tolist(), speed_deg_s=self.args.speed)
        report['preflight'] = self.preflight(q0, q1)
        print(f'Preflight passed: {json.dumps(report)}', flush=True)
        if self.args.check_only:
            return report
        self.publisher = self.create_publisher(JointVelocity, self.command_topic, 10)
        print(f'Moving {self.robot} to {label}: Fabric Follow (CBF) on for {self.robot}, release and press the deadman, '
              'then hold it', flush=True)
        try:
            q = self.drive(q0, q1)
        finally:
            for _ in range(int(RATE_HZ)):
                self.send(np.zeros(6))
                self.spin_for(1.0 / RATE_HZ)
        report['final_joints'] = np.round(q, 4).tolist()
        report['final_error_deg'] = round(float(np.max(np.abs(np.degrees(delta(q, q1))))), 2)
        print(f'Done: largest joint error {report["final_error_deg"]:.2f} deg', flush=True)
        return report


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', choices=('blue', 'silver'), required=True)
    ap.add_argument('--to', nargs='+', default=['grasp'], help='grasp (default), home, or six joint angles (rad)')
    ap.add_argument('--speed', type=float, default=10.0, help='largest joint speed, deg/s (the guard allows 15)')
    ap.add_argument('--check-only', action='store_true', help='run the preflight checks and send nothing')
    args = ap.parse_args()
    if not 0 < args.speed <= 15:
        ap.error('--speed must be in (0, 15] deg/s')
    out = Path.home() / 'fabric_ws' / 'run' / 'grasp_joints'
    out.mkdir(parents=True, exist_ok=True)
    stem = out / f'{args.robot}_{datetime.now():%Y%m%d_%H%M%S}'
    rclpy.init()
    node = GraspJoints(args)
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
                w.writerow(['t', 's', 'forwarded', *[f'v{i}' for i in range(1, 7)], *[f'q{i}' for i in range(1, 7)],
                            'fx', 'fy', 'fz'])
                w.writerows(node.samples)
        Path(f'{stem}.json').write_text(json.dumps(report, indent=1))
        print(f'Saved {stem}.json', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
