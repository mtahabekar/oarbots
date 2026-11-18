#!/usr/bin/env python3
"""Tool orientation during first_motion.py runs (translation commands, zero angular velocity).

Forward kinematics j2n6s300_link_base -> j2n6s300_end_effector from the robot description, applied to
the joint angles each run recorded (samples.csv, q1..q6). Prints per run the tool's travel and its
rotation from the first sample (p50, p99, max). Used for the grip-frame plan (test guide section 9):
the CBF bridge sends no angular velocity, so the grip offset only has to hold at the grasp orientation.

Run on the laptop, from the repository root, with ROS and the workspace sourced (xacro must find
oarbot_description; urdf_parser_py comes with ROS):
    source /opt/ros/jazzy/setup.bash && source ~/ros2_colcon_ws/install/setup.bash
    python3 scripts/hw/h3_orientation.py experiments/hardware/sessions/2026-11-1*_*h3*/*/
blue_20261118_102137 is the ground-loop run (stale joint readings, then jumps): ignore its maximum.
"""
import argparse
import csv
import subprocess
import sys
from pathlib import Path

import numpy as np

XACRO = 'src/oarbot_description/urdf/kinova/j2n6s300_with_force_torque_standalone.xacro'


def axis_angle(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * k @ k


def rpy(r, p, y):
    return axis_angle([0, 0, 1], y) @ axis_angle([0, 1, 0], p) @ axis_angle([1, 0, 0], r)


def chain(robot, base, tip):
    by_child = {j.child: j for j in robot.joints}
    joints, link = [], tip
    while link != base:
        joints.append(by_child[link])
        link = joints[-1].parent
    return joints[::-1]


def fk(joints, q):
    T, qi = np.eye(4), iter(q)
    for j in joints:
        A = np.eye(4)
        if j.origin is not None:
            A[:3, :3] = rpy(*(j.origin.rpy or [0, 0, 0]))
            A[:3, 3] = j.origin.xyz or [0, 0, 0]
        T = T @ A
        if j.type in ('revolute', 'continuous'):
            R = np.eye(4)
            R[:3, :3] = axis_angle(j.axis, next(qi))
            T = T @ R
    return T


def rotation_deg(Ra, Rb):
    return np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


def load_robot(urdf):
    from urdf_parser_py.urdf import URDF
    text = Path(urdf).read_text() if urdf else subprocess.run(
        ['xacro', XACRO], check=True, capture_output=True, text=True).stdout
    stderr, sys.stderr = sys.stderr, open('/dev/null', 'w')   # urdf_parser_py warns on transmission tags
    try:
        return URDF.from_xml_string(text)
    finally:
        sys.stderr.close()
        sys.stderr = stderr


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('runs', nargs='+', help='first_motion run folders (with samples.csv)')
    parser.add_argument('--urdf', help='a URDF file instead of running xacro on the standalone arm')
    args = parser.parse_args()
    joints = chain(load_robot(args.urdf), 'j2n6s300_link_base', 'j2n6s300_end_effector')
    for run in args.runs:
        path = Path(run) / 'samples.csv'
        if not path.exists():
            continue
        rows = [r for r in csv.DictReader(path.open()) if r['q1'] not in ('', 'nan')]
        if not rows:
            continue
        T = [fk(joints, [float(r[f'q{i}']) for i in range(1, 7)]) for r in rows]
        rot = np.array([rotation_deg(T[0][:3, :3], t[:3, :3]) for t in T])
        travel = max(np.linalg.norm(t[:3, 3] - T[0][:3, 3]) for t in T)
        axes = ''.join(sorted({'xyz'[int(r['axis'])] for r in rows if r['axis'] not in ('', 'nan')}))
        print(f'{Path(run).name}: axes {axes or "-"}, travel {travel * 100:.1f} cm, tool rotation '
              f'p50 {np.percentile(rot, 50):.2f} / p99 {np.percentile(rot, 99):.2f} / max {rot.max():.2f} deg')


if __name__ == '__main__':
    main()
