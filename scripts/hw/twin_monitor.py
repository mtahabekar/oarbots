#!/usr/bin/env python3
"""Go/no-go readout for a fabric run: twin, hands, cameras, commands, bridges, deadman (read-only).

Run on the laptop in the session environment while compute.launch.py runs:
    source scripts/hw/laptop_env.sh
    python3 scripts/hw/twin_monitor.py            # one 15 s window
    python3 scripts/hw/twin_monitor.py 60 --every 5   # a line every 5 s for a minute (during a run)

Before switching commands on, want: twin "following the measured grips", stale grips a few % over the
window, all three cameras at ~15 Hz, both hands detected by some camera, deadman released. During a run
watch the bridges ("Following the fabric twin" or the reason they stopped) and the commands.
A new node on the laptop can take up to a minute to discover the camera hosts: use >= 15 s windows.
"""
import argparse
import collections
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from fabric_state_msgs.msg import GloveObservations, HandStates
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

CAMERAS = (('/nuc/fabric/gloves', 'overhead'), ('/oarbot_blue/azure_kinect/fabric/gloves', 'blue_wrist'),
           ('/oarbot_silver/azure_kinect/fabric/gloves', 'silver_wrist'))


class Monitor(Node):
    def __init__(self):
        super().__init__('twin_monitor')
        self.reset()
        self.twin, self.bridges, self.deadman, self.cmd = {}, {}, None, {}
        self.create_subscription(DiagnosticArray, '/fabric/twin/status', self.on_twin, 10)
        for topic, cam in CAMERAS:
            self.create_subscription(GloveObservations, topic, lambda m, cam=cam: self.on_gloves(cam, m), qos_profile_sensor_data)
        self.create_subscription(HandStates, '/fabric/hand_states', self.on_hands, qos_profile_sensor_data)
        for r in ('blue', 'silver'):
            self.create_subscription(DiagnosticArray, f'/oarbot_{r}/diagnostics', lambda m, r=r: self.on_bridge(r, m), 10)
            self.create_subscription(TwistStamped, f'/oarbot_{r}/fabric_cbf/cmd_world', lambda m, r=r: self.on_cmd(r, m), 10)
        self.create_subscription(Bool, '/tablet_gui/deadman', lambda m: setattr(self, 'deadman', m.data), 10)

    def reset(self):
        self.gloves, self.glove_hands, self.hands = collections.Counter(), collections.Counter(), collections.Counter()
        self.first_twin = None

    def on_twin(self, msg):
        for s in msg.status:
            values = {k.key: k.value for k in s.values}
            values['message'] = s.message
            self.twin = values
            if self.first_twin is None and 'frame' in values:
                self.first_twin = values

    def on_gloves(self, cam, msg):
        self.gloves[cam] += 1
        self.glove_hands.update(f'{cam}:{h.hand_id}' for h in msg.hands)

    def on_hands(self, msg):
        self.hands['messages'] += 1
        for h in msg.hands:
            if len(h.source_cameras):
                self.hands[h.hand_id] += 1

    def on_bridge(self, robot, msg):
        for s in msg.status:
            if s.name.endswith('fabric_cbf_bridge'):
                self.bridges[robot] = s.message

    def on_cmd(self, robot, msg):
        v = msg.twist.linear
        self.cmd[robot] = 100 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def report(self, seconds):
        t, a = self.twin, self.first_twin or {}
        try:
            frames = int(t.get('frame', 0)) - int(a.get('frame', 0))
            stale = int(t.get('stale_grip_frames', 0)) - int(a.get('stale_grip_frames', 0))
            stale_txt = f'{frames} frames, stale grip {100 * stale / frames:.0f} %' if frames > 0 else 'no frames'
        except ValueError:
            stale_txt = '-'
        short = lambda key: (t.get(key) or '')[:5]  # noqa: E731
        print(f"twin: {t.get('message', 'no status')} | {stale_txt} | commands {t.get('commands_enabled')} | "
              f"frame p95 {short('frame_ms_p95')} ms, error p95 {short('error_p95_m')} m, box clearance "
              f"{short('box_clearance_m')} m, CBF {t.get('cbf_status')}", flush=True)
        cams = ', '.join(f"{cam} {self.gloves[cam] / seconds:.1f} Hz (left {100 * self.glove_hands[cam + ':human_left'] / max(self.gloves[cam], 1):.0f} %, "
                         f"right {100 * self.glove_hands[cam + ':human_right'] / max(self.gloves[cam], 1):.0f} %)" for _, cam in CAMERAS)
        n = max(self.hands['messages'], 1)
        print(f"gloves: {cams}", flush=True)
        print(f"fused hands fresh: left {100 * self.hands['human_left'] / n:.0f} %, right {100 * self.hands['human_right'] / n:.0f} % | "
              f"commands cm/s {({r: round(v, 1) for r, v in self.cmd.items()})} | bridges {self.bridges} | deadman {self.deadman}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('seconds', type=float, nargs='?', default=15.0)
    ap.add_argument('--every', type=float, default=0.0, help='print a report every N seconds (each its own window)')
    args = ap.parse_args()
    rclpy.init()
    node = Monitor()
    try:
        end = time.time() + args.seconds
        window = args.every or args.seconds
        while time.time() < end:
            start = time.time()
            node.reset()
            while time.time() < min(start + window, end):
                rclpy.spin_once(node, timeout_sec=0.05)
            print(f'--- {time.strftime("%H:%M:%S")} ({time.time() - start:.0f} s window)', flush=True)
            node.report(max(time.time() - start, 1e-3))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
