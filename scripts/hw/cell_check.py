#!/usr/bin/env python3
"""Readiness of the arms, guards and bridges, from the laptop (read-only; `fabric_run.sh check` runs it).

For each robot: where the arm is against its grasp pose (grasp.yaml) and its box (bridge.yaml), the fingers,
the command guard's state and joint cap, and the fabric bridge's state and settings (force stop, box stop).
    python3 scripts/hw/cell_check.py [seconds]
"""
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import JointState

CELL = Path(__file__).resolve().parents[2] / 'experiments' / 'hardware' / 'cell'


def param(node, name):
    try:
        out = subprocess.run(['ros2', 'param', 'get', node, name], capture_output=True, text=True, timeout=15).stdout
        return out.strip().split(':')[-1].strip() or '?'
    except subprocess.TimeoutExpired:
        return '?'


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 4.
    grasp = yaml.safe_load((CELL / 'grasp.yaml').read_text())
    bridge_cfg = yaml.safe_load((CELL / 'bridge.yaml').read_text())
    rclpy.init()
    n = Node('cell_check')
    q, guard, bridge = {}, {}, {}
    for r in ('blue', 'silver'):
        n.create_subscription(JointState, f'/oarbot_{r}/joint_states', lambda m, r=r: q.__setitem__(r, list(m.position)), 10)

        def diag(m, r=r):
            for s in m.status:
                if s.name.endswith('command_guard'):
                    guard[r] = s.message
                elif s.name.endswith('fabric_cbf_bridge'):
                    bridge[r] = (s.message, {k.key: k.value for k in s.values})
        n.create_subscription(DiagnosticArray, f'/oarbot_{r}/diagnostics', diag, 10)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    for r in ('blue', 'silver'):
        print(f'== {r}')
        if r in q:
            d = [math.degrees(abs(math.atan2(math.sin(a - b), math.cos(a - b)))) for a, b in zip(q[r][:6], grasp[r])]
            fingers = [round(v, 2) for v in q[r][6:9]]
            where = 'at the grasp pose' if max(d) < 3 else f'{max(d):.0f} deg from the grasp pose (fabric_run.sh recentre)'
            print(f'  arm: {where}; fingers {fingers} ({"closed" if max(fingers or [0]) > 1. else "open / not driven"})')
        else:
            print('  arm: NO joint states')
        print(f'  guard: {guard.get(r, "NO diagnostics")}; joint cap {param(f"/oarbot_{r}/command_guard", "max_arm_joint_dps")} deg/s')
        if r in bridge:
            msg, v = bridge[r]
            box = {**bridge_cfg['common'], **bridge_cfg.get(r, {})}
            print(f'  bridge: {msg}; output {v.get("output")}, force stop '
                  f'{"off" if float(v.get("force_limit_n", 0)) == 0 else v.get("force_limit_n") + " N"}'
                  f' (change since arming {v.get("force_change_n") or "-"} N), box stop {v.get("workspace_stop")}')
            print(f'  box (bridge.yaml): {box["workspace_min"]} .. {box["workspace_max"]}, '
                  f'{box["max_speed_mps"]*100:.0f} cm/s, {box["max_joint_dps"]:.0f} deg/s')
        else:
            print('  bridge: not running (fabric_run.sh bridges start)')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
