#!/usr/bin/env python3
"""Start the closed loop at the first deadman press, for an operator who cannot see the screen.

`fabric_run.sh go` runs this after starting the twin (commands off). It disarms both bridges and keeps
commands off, then waits. The laptop speaks (spd-say):
  - "twin ready" once the twin follows the measured grips (and what it waits for, every 30 s, until then);
  - at the first deadman press while the twin is ready, it switches the twin's commands on (the formation is
    captured at that moment: hold the sheet as you want it kept) and arms both bridges: "commands on";
  - a press while the twin is not ready does nothing but say why;
  - after that, "moving" / "stopped" at each press / release, and any bridge that stops, with its reason.
Ctrl+C (or the end of --seconds): commands off, bridges disarmed. Events go to <run_dir>/auto_start.log.
    python3 scripts/hw/auto_start.py [--run-dir DIR] [--seconds 1800] [--quiet]
"""
import argparse
import subprocess
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from std_msgs.msg import Bool

READY = 'twin following the measured grips'
ROBOTS = ('blue', 'silver')


class AutoStart(Node):
    def __init__(self, log, quiet):
        super().__init__('fabric_auto_start')
        self.log, self.quiet = log, quiet
        self.enable_pub = self.create_publisher(Bool, '/fabric/twin/enable_commands', 1)
        self.arm_pubs = {r: self.create_publisher(Bool, f'/oarbot_{r}/fabric_cbf/arm', 1) for r in ROBOTS}
        self.twin, self.twin_seen, self.bridges = '', 0., {}
        self.deadman, self.started = None, False
        self.last_wait_say = 0.
        self.create_subscription(DiagnosticArray, '/fabric/twin/status', self.on_twin, 10)
        for r in ROBOTS:
            self.create_subscription(DiagnosticArray, f'/oarbot_{r}/diagnostics', lambda m, r=r: self.on_bridge(r, m), 10)
        self.create_subscription(Bool, '/tablet_gui/deadman', self.on_deadman, 10)
        self.create_timer(1., self.tick)

    def event(self, text, say=None):
        line = f'{time.strftime("%H:%M:%S")} {text}'
        print(line, flush=True)
        with self.log.open('a') as f:
            f.write(line + '\n')
        if say and not self.quiet:
            subprocess.Popen(['spd-say', '-r', '-10', say], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def send(self, enable):
        """Commands on/off and the bridges armed/disarmed, sent a few times (single messages can be missed)."""
        for _ in range(5):
            self.enable_pub.publish(Bool(data=enable))
            for pub in self.arm_pubs.values():
                pub.publish(Bool(data=enable))
            time.sleep(.1)

    def ready(self):
        return self.twin == READY and time.monotonic() - self.twin_seen < 3.

    def on_twin(self, msg):
        for s in msg.status:
            if s.message != self.twin:
                self.event(f'twin: {s.message}', 'twin ready. Press the deadman when you hold the sheet'
                           if s.message == READY and not self.started else None)
            self.twin, self.twin_seen = s.message, time.monotonic()

    def on_bridge(self, robot, msg):
        for s in msg.status:
            if not s.name.endswith('fabric_cbf_bridge') or self.bridges.get(robot) == s.message:
                continue
            old, self.bridges[robot] = self.bridges.get(robot), s.message
            stopped = self.started and old is not None and s.message != 'Following the fabric twin'
            self.event(f'{robot} bridge: {s.message}', f'{robot} stopped. {s.message}' if stopped else None)

    def on_deadman(self, msg):
        pressed, was = bool(msg.data), self.deadman
        self.deadman = pressed
        if was is None or pressed == was:
            return
        if pressed and not self.started:
            if self.ready():
                self.send(True)
                self.started = True
                self.event('deadman pressed: commands ON, bridges armed (formation captured now)', 'commands on')
            else:
                why = self.twin or 'no twin status'
                self.event(f'deadman pressed, twin not ready: {why}', f'not ready. {why}')
        elif self.started:
            self.event('deadman ' + ('pressed: moving' if pressed else 'released: arms stopped'), 'moving' if pressed else 'stopped')

    def tick(self):
        if not self.started and not self.ready() and time.monotonic() - self.last_wait_say > 30.:
            self.last_wait_say = time.monotonic()
            self.event(f'waiting: {self.twin or "no twin status yet"}', 'waiting for the twin')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--run-dir', default='.')
    ap.add_argument('--seconds', type=float, default=1800.)
    ap.add_argument('--quiet', action='store_true', help='no speech')
    a = ap.parse_args()
    rclpy.init()
    node = AutoStart(Path(a.run_dir) / 'auto_start.log', a.quiet)
    node.send(False)
    node.event('commands off, bridges disarmed; waiting for the twin and the first deadman press')
    end = time.monotonic() + a.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(False)
        node.event('end: commands off, bridges disarmed', 'commands off')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
