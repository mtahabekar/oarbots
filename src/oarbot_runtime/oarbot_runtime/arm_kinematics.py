"""Serial-chain kinematics from a URDF, for resolved-rate (joint velocity) control of a grip point.

Kinova's firmware refuses Cartesian velocity in some arm configurations (CartesianFaultState 1 at the
fabric grasp pose, 2026-11-18), while joint velocity works there. The fabric bridge therefore turns its
safe Cartesian grip velocity into joint rates with the robot's own model: the same URDF and joint states
that produce the TF everything else uses, so commanded and measured motion share one model.
"""
from math import cos, sin

import numpy as np


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
    return np.eye(3) + sin(angle) * k + (1. - cos(angle)) * k @ k


def rpy_matrix(roll, pitch, yaw):
    return axis_angle_matrix([0, 0, 1], yaw) @ axis_angle_matrix([0, 1, 0], pitch) @ axis_angle_matrix([1, 0, 0], roll)


def rotation_vector(rotation):
    """Axis times angle of a rotation matrix (the matrix logarithm), for small orientation errors."""
    c = np.clip((np.trace(rotation) - 1.) / 2., -1., 1.)
    angle = np.arccos(c)
    w = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])
    if angle < 1e-6:
        return w / 2.
    return w * angle / (2. * np.sin(angle))


class Chain:
    """Revolute/continuous/fixed joints from `base` to `tip` in a URDF (urdf_parser_py robot or XML)."""

    def __init__(self, robot, base, tip):
        if isinstance(robot, str):
            from urdf_parser_py.urdf import URDF
            robot = URDF.from_xml_string(robot)
        by_child = {j.child: j for j in robot.joints}
        joints, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f'No joint chain from {base} to {tip} (stopped at {link})')
            joints.append(by_child[link])
            link = joints[-1].parent
        self.joints = joints[::-1]
        self.names = [j.name for j in self.joints if j.type in ('revolute', 'continuous')]
        if any(j.type not in ('revolute', 'continuous', 'fixed') for j in self.joints):
            raise ValueError('Only revolute, continuous and fixed joints are supported')
        self.origins = []
        for j in self.joints:
            T = np.eye(4)
            if j.origin is not None:
                T[:3, :3] = rpy_matrix(*(j.origin.rpy or [0., 0., 0.]))
                T[:3, 3] = j.origin.xyz or [0., 0., 0.]
            self.origins.append(T)

    def frames(self, q, tip_offset=None):
        """Tip pose (4x4, in `base`) and each moving joint's axis and origin in `base`."""
        q = list(q)
        if len(q) != len(self.names):
            raise ValueError(f'Expected {len(self.names)} joint values')
        T, axes, points, index = np.eye(4), [], [], 0
        for j, origin in zip(self.joints, self.origins):
            T = T @ origin
            if j.type != 'fixed':
                axis = T[:3, :3] @ np.asarray(j.axis, float)
                axes.append(axis / np.linalg.norm(axis))
                points.append(T[:3, 3].copy())
                M = np.eye(4)
                M[:3, :3] = axis_angle_matrix(j.axis, q[index])
                T = T @ M
                index += 1
        if tip_offset is not None:
            T = T @ tip_offset
        return T, np.array(axes), np.array(points)

    def fk(self, q, tip_offset=None):
        return self.frames(q, tip_offset)[0]

    def jacobian(self, q, tip_offset=None):
        """Geometric Jacobian (6 x n) of the tip point: rows linear velocity, then angular, in `base`."""
        T, axes, points = self.frames(q, tip_offset)
        tip = T[:3, 3]
        return np.vstack([np.cross(axes, tip - points).T, axes.T]), T


def joint_rates(jacobian, twist, damping=0.01):
    """Damped least-squares joint rates for a 6-vector twist (linear, angular) in the Jacobian's frame.

    The damping (in metres and radians) bounds the rates near a singularity at the cost of a small
    tracking error there; away from one it changes the solution negligibly."""
    J = np.asarray(jacobian, float)
    return J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(J.shape[0]), np.asarray(twist, float))


def scale_to_limit(rates, limit):
    """Scale all joint rates together so none exceeds `limit` (keeps the tip's direction)."""
    peak = float(np.max(np.abs(rates))) if len(rates) else 0.
    return rates * min(1., limit / peak) if peak > 0 else rates


class GripRates:
    """The bridge's Cartesian grip velocity as joint rates, holding the tool orientation it had at arming.

    `rates(q, linear)` takes the joint positions (chain order, rad) and the grip's linear velocity in the
    chain's base frame (m/s), and returns joint rates (rad/s) with every joint at most `max_rate`. The
    angular part of the twist is `gain` times the rotation from the current tool orientation back to the
    reference: the bridge never asks for rotation, so drift from integration or the model is pulled back."""

    def __init__(self, chain, max_rate, gain=1.0, damping=0.01):
        if not (max_rate > 0 and gain >= 0 and damping > 0):
            raise ValueError('Invalid joint-rate settings')
        self.chain, self.max_rate, self.gain, self.damping = chain, max_rate, gain, damping
        self.reference = None

    def reset(self):
        self.reference = None

    def rates(self, q, linear, tip_offset=None):
        J, T = self.chain.jacobian(q, tip_offset)
        if self.reference is None:
            self.reference = T[:3, :3].copy()
        hold = self.gain * rotation_vector(self.reference @ T[:3, :3].T)
        return scale_to_limit(joint_rates(J, np.r_[np.asarray(linear, float), hold], self.damping), self.max_rate)

    def orientation_error(self, q, tip_offset=None):
        """Angle (rad) between the current tool orientation and the reference, or 0 without one."""
        if self.reference is None:
            return 0.
        T = self.chain.fk(q, tip_offset)
        return float(np.linalg.norm(rotation_vector(self.reference @ T[:3, :3].T)))
