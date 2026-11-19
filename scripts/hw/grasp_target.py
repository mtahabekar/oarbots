#!/usr/bin/env python3
"""Where a grasp pose puts a real robot's tool and wrist camera, and whether MoveIt finds it free.

Read-only: sends nothing to the arm. Uses the robot's own MoveIt (compute_fk, check_state_validity)
in the running session, the parked base pose from the cell calibration and the wrist camera's
intrinsics, and reports:
  - the tool (j2n6s300_end_effector) in the world now and at the target joints: position, the
    direction it points (pitch below horizontal, azimuth from +x), and the change from now;
  - collisions at the target and along a straight joint-space path from the current joints;
  - where the cell's key points fall in the wrist camera's image at the target (own and other grip,
    the two hand grips, the sheet's centre), so you can see what the camera will look at.

Run on the laptop in the session environment:
    source scripts/hw/laptop_env.sh
    python3 scripts/hw/grasp_target.py --robot blue          # targets from experiments/hardware/cell/grasp.yaml
    python3 scripts/hw/grasp_target.py --robot blue --current  # the pose the arm is in now
    python3 scripts/hw/grasp_target.py --robot blue --position -0.40 -0.42 1.05
        # the tool moved to a world position with its current orientation (translation only), by IK
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState
from tf2_ros import Buffer, TransformListener

REPO = Path(__file__).resolve().parents[2]
CELL = REPO / 'experiments' / 'hardware' / 'cell'
ARM = [f'j2n6s300_joint_{i}' for i in range(1, 7)]
REVOLUTE = {1, 2}          # joints 2 and 3 (index 1, 2) have limits; the others are continuous
SHEET = 0.84               # sheet side (m); hands hold the far corners, ~1.05 m high (glove test)
HAND_Z = 1.05


def quat_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_quat(R):
    w = math.sqrt(max(0., 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0., 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0., 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0., 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def pose_matrix(p):
    T = np.eye(4)
    T[:3, :3] = quat_matrix(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def base_poses():
    cal = yaml.safe_load((CELL / 'calibration.yaml').read_text())
    out = {}
    for t in cal['transforms']:
        if t['parent'] == 'world' and t['child'].endswith('/base_link'):
            T = np.eye(4)
            T[:3, :3] = quat_matrix(*t['rotation_xyzw'])
            T[:3, 3] = t['translation']
            out[t['child'].split('/')[0].replace('oarbot_', '')] = T
    return out


def direction(R, axis):
    d = R[:, axis]
    pitch = math.degrees(math.asin(max(-1., min(1., -d[2]))))
    return d, pitch, math.degrees(math.atan2(d[1], d[0]))


def path(q0, q1, steps):
    d = np.array(q1) - np.array(q0)
    for i in range(6):
        if i not in REVOLUTE:
            d[i] = (d[i] + math.pi) % (2 * math.pi) - math.pi
    return [list(np.array(q0) + d * k / steps) for k in range(steps + 1)], d


class Target(Node):
    def __init__(self, robot):
        super().__init__('grasp_target')
        self.robot, self.ns = robot, f'/oarbot_{robot}'
        self.joints, self.info = None, None
        self.create_subscription(JointState, self.ns + '/joint_states', self.on_joints, 10)
        self.create_subscription(CameraInfo, self.ns + '/azure_kinect/rgb/camera_info', self.on_info, 10)
        self.fk = self.create_client(GetPositionFK, self.ns + '/compute_fk')
        self.valid = self.create_client(GetStateValidity, self.ns + '/check_state_validity')
        self.ik_client = self.create_client(GetPositionIK, self.ns + '/compute_ik')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def on_joints(self, msg):
        if any(n.endswith('j2n6s300_joint_1') for n in msg.name):
            self.joints = msg

    def on_info(self, msg):
        self.info = msg

    def wait(self, pred, what, sec=15.):
        end = self.get_clock().now().nanoseconds + sec * 1e9
        while not pred():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > end:
                raise SystemExit(f'No {what} after {sec:.0f} s')

    def call(self, client, req):
        self.wait(client.service_is_ready, client.srv_name)
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.)
        if fut.result() is None:
            raise SystemExit(f'{client.srv_name} did not answer')
        return fut.result()

    def state(self, arm):
        js = JointState()
        js.name = list(self.joints.name)
        js.position = list(self.joints.position)
        for i, n in enumerate(ARM):
            js.position[js.name.index(f'oarbot_{self.robot}/{n}')] = float(arm[i])
        # Fingers open for the check: closed fingertips touch each other in the model (Silver's are held
        # closed), which MoveIt would count as a collision of every arm pose.
        js.position = [0.0 if 'finger' in name else v for name, v in zip(js.name, js.position)]
        rs = RobotState()
        rs.joint_state = js
        return rs

    def links(self, arm, names):
        req = GetPositionFK.Request()
        req.header.frame_id = f'oarbot_{self.robot}/base_link'
        req.fk_link_names = [f'oarbot_{self.robot}/{n}' for n in names]
        req.robot_state = self.state(arm)
        res = self.call(self.fk, req)
        if res.error_code.val != 1:
            raise SystemExit(f'compute_fk failed (MoveIt code {res.error_code.val})')
        return {n: pose_matrix(p.pose) for n, p in zip(names, res.pose_stamped)}

    def ik(self, seed, T_base):
        """Joints that put the tool at T_base (in base_link), seeded from `seed`; None if MoveIt finds none."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = 'arm'
        r.robot_state = self.state(seed)
        r.avoid_collisions = True
        r.ik_link_name = f'oarbot_{self.robot}/j2n6s300_end_effector'
        r.pose_stamped.header.frame_id = f'oarbot_{self.robot}/base_link'
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = (float(v) for v in T_base[:3, 3])
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = matrix_quat(T_base[:3, :3])
        r.timeout.sec = 2
        res = self.call(self.ik_client, req)
        if res.error_code.val != 1:
            return None
        names = list(res.solution.joint_state.name)
        return [res.solution.joint_state.position[names.index(f'oarbot_{self.robot}/{n}')] for n in ARM]

    def contacts(self, arm):
        req = GetStateValidity.Request()
        req.robot_state = self.state(arm)
        req.group_name = 'arm'
        res = self.call(self.valid, req)
        return res.valid, sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts})

    def current(self):
        self.wait(lambda: self.joints is not None, 'joint states')
        return [self.joints.position[self.joints.name.index(f'oarbot_{self.robot}/{n}')] for n in ARM]

    def camera_from_base(self):
        """camera_base -> rgb_camera_link (the driver's static transform)."""
        self.wait(lambda: self.tf.can_transform(f'oarbot_{self.robot}/camera_base',
                                                f'oarbot_{self.robot}/rgb_camera_link', rclpy.time.Time()),
                  'camera_base -> rgb_camera_link transform')
        t = self.tf.lookup_transform(f'oarbot_{self.robot}/camera_base', f'oarbot_{self.robot}/rgb_camera_link',
                                     rclpy.time.Time()).transform
        T = np.eye(4)
        T[:3, :3] = quat_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return T


def grip_points(bases, tools):
    """Cell key points in the world: robot grips (at the given tools), hand grips across the sheet."""
    pts = {}
    for r, p in tools.items():
        pts[f'{r} grip'] = p
    if len(tools) == 2:
        b, s = tools['blue'], tools['silver']
        along = np.array([0., 1., 0.])      # robots face +y, the person stands at +y
        pts['right hand grip (tag 20)'] = np.array([b[0], b[1] + SHEET - 0.03, HAND_Z])
        pts['left hand grip (tag 21)'] = np.array([s[0], s[1] + SHEET - 0.03, HAND_Z])
        pts['sheet centre (hanging ~0.15 m)'] = (b + s) / 2 + along * SHEET / 2 - np.array([0, 0, 0.15])
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--robot', choices=('blue', 'silver'), required=True)
    ap.add_argument('--joints', type=float, nargs=6, help='target joints (rad); default from grasp.yaml')
    ap.add_argument('--current', action='store_true', help='report the current pose only')
    ap.add_argument('--position', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                    help='target tool position in the world, current orientation kept (IK)')
    ap.add_argument('--steps', type=int, default=40, help='collision checks along the joint path')
    args = ap.parse_args()
    grasp = yaml.safe_load((CELL / 'grasp.yaml').read_text())
    bases = base_poses()
    rclpy.init()
    node = Target(args.robot)
    try:
        now = node.current()
        W = bases[args.robot]
        target = now if args.current else (args.joints or grasp[args.robot])
        if args.position:
            T = node.links(now, ['j2n6s300_end_effector'])['j2n6s300_end_effector']
            T[:3, 3] = (np.linalg.inv(W) @ np.append(args.position, 1.))[:3]
            target = node.ik(now, T)
            if target is None:
                raise SystemExit(f'MoveIt finds no collision-free IK for the tool at {args.position} with its current orientation')
        L = {k: W @ v for k, v in node.links(target, ['j2n6s300_end_effector', 'camera_base']).items()}
        L0 = {k: W @ v for k, v in node.links(now, ['j2n6s300_end_effector']).items()}
        tool, tool0 = L['j2n6s300_end_effector'], L0['j2n6s300_end_effector']
        print(f'{args.robot}: current joints {np.round(now, 3).tolist()}')
        print(f'{args.robot}: target  joints {np.round(target, 3).tolist()}')
        for name, T in (('now', tool0), ('target', tool)):
            d, pitch, az = direction(T[:3, :3], 2)
            print(f'  tool {name:6s} world ({T[0, 3]:+.3f}, {T[1, 3]:+.3f}, {T[2, 3]:+.3f}) m, '
                  f'points {pitch:+.0f} deg below horizontal, azimuth {az:+.0f} deg; '
                  f'{np.linalg.norm(T[:2, 3] - W[:2, 3]):.3f} m from the base centre (horizontal)')
        if not args.current:
            print(f'  tool moves {np.linalg.norm(tool[:3, 3] - tool0[:3, 3]):.3f} m')
            ok, bad = node.contacts(target)
            print(f'  MoveIt at the target: {"free" if ok else "COLLISION " + str(bad)}')
            steps, d = path(now, target, args.steps)
            print(f'  joint change (shortest way for the continuous joints): {np.round(np.degrees(d)).astype(int).tolist()} deg')
            hits = [(k, node.contacts(q)[1]) for k, q in enumerate(steps)]
            hits = [(k, c) for k, c in hits if c]
            print(f'  straight joint path, {args.steps} steps: ' +
                  ('free' if not hits else f'collides from step {hits[0][0]} to {hits[-1][0]}: {hits[0][1]}'))
        # the wrist camera's view at the target
        C = L['camera_base'] @ node.camera_from_base()
        optical = C[:3, 2]
        pitch = math.degrees(math.asin(max(-1., min(1., -optical[2]))))
        print(f'  wrist camera at ({C[0, 3]:+.3f}, {C[1, 3]:+.3f}, {C[2, 3]:+.3f}), looking {pitch:+.0f} deg below '
              f'horizontal, azimuth {math.degrees(math.atan2(optical[1], optical[0])):+.0f} deg')
        node.wait(lambda: node.info is not None, 'camera_info', 10.)
        K = np.array(node.info.k).reshape(3, 3)
        w, h = node.info.width, node.info.height
        tools = {args.robot: tool[:3, 3]}
        other = 'silver' if args.robot == 'blue' else 'blue'
        tools[other] = np.array([-tool[0, 3] if abs(bases[other][0, 3] + W[0, 3]) < 0.05 else tool[0, 3],
                                 tool[1, 3], tool[2, 3]])       # mirror image across x = 0
        Cinv = np.linalg.inv(C)
        for name, p in grip_points(bases, tools).items():
            q = Cinv @ np.append(p, 1.)
            if q[2] <= 0:
                print(f'    {name:32s} behind the camera')
                continue
            u, v = (K @ (q[:3] / q[2]))[:2]
            inside = 0 <= u < w and 0 <= v < h
            print(f'    {name:32s} pixel ({u:6.0f}, {v:6.0f}) {"in view" if inside else "out of view"}, {q[2]:.2f} m away')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
