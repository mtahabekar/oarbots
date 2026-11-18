#!/usr/bin/env python3
"""Drive the Dingo bases to their targets with the overhead camera as feedback, through the guard.

Runs on the NUC next to `park_guide.py serve` (which writes the bases' poses to /dev/shm/park.json
from every overhead frame). It publishes, on the session domain, each robot's `base_enable` and
`commands/teleop/base`, the command guard's teleop base input. Nothing moves unless the operator
holds the tablet deadman: the guard passes base commands only with a fresh, armed deadman, a clear
E-stop and a fresh `base_enable`. Run it with the fabric GUI closed; the tablet runs only its
deadman/E-stop publisher, so this is the only publisher on those inputs (the guard trips on two).

One robot at a time, in the order given (Silver first when it is in Blue's way). Plain `robot`:
turn in place to the target heading, then translate (holonomic) holding it. `robot@x,y`: translate
to the staging point (x, y) without turning, turn there, then translate to the target holding the
heading; use it when the robot (or an arm sticking out) must not turn where it stands or next to
its parked neighbour. Final goal: within 2 cm and 2 degrees for 1.5 s (or 3 cm, 3 degrees for 6 s); staging: 4 cm, 3 degrees.
At most 0.05 m/s and 0.10 rad/s (guard limits 0.15 / 0.3); zero whenever the pose is older than
1.7 s (the overhead frames arrive 1.15 s late) or the marker is not seen.

    python3 park_drive.py overhead_world.json oarbot_silver oarbot_blue@-0.42,-0.30

Where it runs. The guard rejects commands stamped in its future, so run it on the machine whose
clock is furthest behind among the helpers (2026-11-17: Silver, 46 ms behind the laptop, Blue 20 ms):
commands then reach every guard with an age of zero or more. Away from the NUC it reads the poses
from `/park/poses` (std_msgs/String with the guide's JSON), relayed by `park_guide.py relay`.

Dingo E-stop. Each Dingo's own platform driver reports its E-stop on `dingo/platform/emergency_stop`
and twist_mux locks on it. The stock tablet GUI forwards the tablet's E-stop button there (false
when clear); with the GUI closed this does the same, and sends true when the tablet's E-stop
message is older than 0.25 s.
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, String

ESTOP_STALE_SEC = 0.25
POSES = {'stamp': None, 'poses': {}}

# The stock overhead camera (5 fps) delivers frames 1.15 s after capture (measured 2026-11-17 against
# the camera's own IMU stream on the same device clock), so the loop is slow: gain 0.3 /s keeps the
# phase lag of that delay near 20 degrees at crossover.
V_MAX, W_MAX, V_MIN, W_MIN = 0.05, 0.10, 0.012, 0.03     # m/s, rad/s; the minima overcome the drive deadband
K_V, K_W = 0.3, 0.4                                       # 1/s
TOL_M, TOL_DEG, TURN_FIRST_DEG = 0.02, 2.0, 4.0
STALE_SEC, SETTLE_SEC, RATE_HZ = 1.7, 1.5, 20.0


def read_poses():
    if not os.path.exists('/dev/shm/park.json'):
        return POSES['stamp'], POSES['poses']               # from /park/poses
    try:
        with open('/dev/shm/park.json') as fh:
            d = json.load(fh)
        return d['stamp'], d['poses']
    except (OSError, ValueError, KeyError):
        return None, {}


def on_poses(msg):
    try:
        d = json.loads(msg.data)
        POSES['stamp'], POSES['poses'] = d['stamp'], d['poses']
    except (ValueError, KeyError):
        pass


class ParkDrive(Node):
    def __init__(self, world, specs):
        super().__init__('park_drive')
        targets = json.load(open(world))['targets']
        self.order, self.goals = [], []          # goals: (robot, x, y, yaw or None, final)
        for spec in specs:
            robot, _, staging = spec.partition('@')
            tx, ty, tyaw = targets[robot]
            self.order.append(robot)
            if staging:
                sx, sy = (float(v) for v in staging.split(','))
                self.goals += [(robot, sx, sy, None, False), (robot, sx, sy, tyaw, False)]
            self.goals.append((robot, tx, ty, tyaw, True))
        self.enable = {r: self.create_publisher(Bool, f'/{r}/base_enable', 1) for r in self.order}
        self.cmd = {r: self.create_publisher(TwistStamped, f'/{r}/commands/teleop/base', 1) for r in self.order}
        self.dingo_estop = {r: self.create_publisher(Bool, f'/{r}/dingo/platform/emergency_stop', 10) for r in self.order}
        self.estop_time = float('-inf')
        self.create_subscription(Bool, '/tablet_gui/e_stop', self.on_estop, 10)
        self.create_subscription(String, '/park/poses', on_poses, 10)
        self.active, self.settled_since, self.phase = 0, None, 'turn'
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.create_timer(0.1, self.estop_watchdog)
        self.last_log = 0.0
        self.log(f'goals {[(g[0], g[1], g[2], g[3]) for g in self.goals]}; hold the tablet deadman to move (release stops)')

    def log(self, text):
        print(time.strftime('%H:%M:%S'), text, flush=True)

    def on_estop(self, msg):
        # Forward the tablet's E-stop to each Dingo, as the stock GUI does.
        self.estop_time = time.monotonic()
        for pub in self.dingo_estop.values():
            pub.publish(Bool(data=msg.data))

    def estop_watchdog(self):
        if time.monotonic() - self.estop_time > ESTOP_STALE_SEC:
            for pub in self.dingo_estop.values():
                pub.publish(Bool(data=True))

    def send(self, robot, vx=0.0, vy=0.0, wz=0.0, enable=True):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{robot}/base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.cmd[robot].publish(msg)
        self.enable[robot].publish(Bool(data=enable))

    def tick(self):
        if self.active >= len(self.goals):
            for r in self.order:
                self.send(r, enable=False)
            return
        robot, tx, ty, tyaw, final = self.goals[self.active]
        for r in self.order:
            if r != robot:
                self.send(r, enable=False)
        stamp, poses = read_poses()
        if stamp is None or robot not in poses or time.time() - stamp > STALE_SEC:
            self.send(robot)                                      # enabled, zero: wait for fresh feedback
            self.throttled(f'{robot}: no fresh pose ({"not seen" if stamp else "no data"}); holding still')
            return
        x, y, yaw = poses[robot]
        dx, dy = tx - x, ty - y
        turn = 0.0 if tyaw is None else (tyaw - yaw + 180) % 360 - 180
        dist = math.hypot(dx, dy)
        tol_m, tol_deg = (TOL_M, TOL_DEG) if final else (0.04, 3.0)
        # Marker noise and the drive deadband can keep a base hunting at the fine tolerance: accept a
        # final pose that has stayed within 3 cm and 3 degrees for 6 s.
        near = dist < 0.03 and abs(turn) < 3.0
        self.near_since = (getattr(self, 'near_since', None) or time.time()) if near else None
        if final and near and time.time() - self.near_since > 6.0:
            tol_m, tol_deg = 0.03, 3.0
        if dist < tol_m and abs(turn) < tol_deg:
            self.send(robot)
            self.settled_since = self.settled_since or time.time()
            if time.time() - self.settled_since > (SETTLE_SEC if final else 0.3):
                self.log(f'{robot}: {"parked" if final else "reached staging"} at ({x:+.3f}, {y:+.3f}) {yaw:+.1f} deg; '
                         f'error {dist*100:.1f} cm, {turn:+.1f} deg')
                self.active, self.settled_since, self.phase = self.active + 1, None, 'turn'
            return
        self.settled_since = None
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        fwd, left = c * dx + s * dy, -s * dx + c * dy
        wz = math.copysign(min(W_MAX, max(W_MIN, K_W * abs(math.radians(turn)))), turn) if abs(turn) >= tol_deg else 0.0
        if self.phase == 'turn' and abs(turn) < TURN_FIRST_DEG:
            self.phase = 'move'
            if dist >= tol_m:
                self.log(f'{robot}: heading {"free" if tyaw is None else "reached"}, translating {dist:.2f} m')
        vx = vy = 0.0
        if self.phase == 'move' and dist >= tol_m:
            speed = min(V_MAX, max(V_MIN, K_V * dist))
            vx, vy = speed * fwd / dist, speed * left / dist
        self.send(robot, vx, vy, wz)
        self.throttled(f'{robot}: {self.phase} | at ({x:+.3f}, {y:+.3f}) {yaw:+.1f} deg | to go {dist*100:.1f} cm, '
                       f'{turn:+.1f} deg | cmd vx {vx:+.3f} vy {vy:+.3f} wz {wz:+.3f}')

    def throttled(self, text):
        if time.time() - self.last_log > 2.0:
            self.log(text)
            self.last_log = time.time()

    def stop(self):
        for _ in range(5):
            for r in self.order:
                self.send(r, enable=False)
                self.dingo_estop[r].publish(Bool(data=True))
            time.sleep(0.05)


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    rclpy.init()
    node = ParkDrive(sys.argv[1], sys.argv[2:])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
