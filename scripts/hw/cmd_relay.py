#!/usr/bin/env python3
"""Relay the fabric twin's follower commands from the simulated cell to the real robots.

The simulated cell (plant, estimator, twin) runs on its own ROS domain with Fast DDS and uses the
same topic and frame names as the robots, so only the two command topics cross, through this relay,
over a UDP socket on this computer:

    sim:  in the simulation's domain. Subscribes /oarbot_{blue,silver}/fabric_cbf/cmd_world and
          /clock, and sends each command with the measured real-time factor.
    real: in the fabric session domain (CycloneDDS). Republishes the newest command for each robot at
          50 Hz on /oarbot_<robot>/fabric_cbf/cmd_world, stamped now (the twin stamps simulated time,
          which the bridges would reject as stale), and scales the velocity by the real-time factor
          so a real arm covers the same distance per second as its simulated copy. A command not
          refreshed within 0.3 s is not repeated, so the bridges fall to zero.

    scripts/hw/run_sim_twin.sh starts both sides with the right environments.
"""
import argparse
import json
import socket
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

ROBOTS = ('blue', 'silver')
PORT = 47018
HOLD_SEC = 0.3


def run_sim(args):
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    node = rclpy.create_node('fabric_cmd_relay_sim')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    clock = []   # (wall, sim) over the last 3 s

    def on_clock(msg):
        now = time.monotonic()
        clock.append((now, msg.clock.sec + msg.clock.nanosec * 1e-9))
        while clock and now - clock[0][0] > 3.0:
            clock.pop(0)

    def rtf():
        if len(clock) < 2 or clock[-1][0] - clock[0][0] < 1.0:
            return None
        return (clock[-1][1] - clock[0][1]) / (clock[-1][0] - clock[0][0])

    def on_command(robot, msg):
        v = msg.twist.linear
        sock.sendto(json.dumps(dict(robot=robot, frame=msg.header.frame_id, v=[v.x, v.y, v.z], rtf=rtf())).encode(),
                    ('127.0.0.1', args.port))

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    for robot in ROBOTS:
        node.create_subscription(TwistStamped, f'/oarbot_{robot}/fabric_cbf/cmd_world',
                                 lambda m, r=robot: on_command(r, m), qos_profile_sensor_data)
    print('Relay (sim side): forwarding the twin commands', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def run_real(args):
    rclpy.init()
    node = rclpy.create_node('fabric_cmd_relay_real')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', args.port))
    sock.setblocking(False)
    pubs = {r: node.create_publisher(TwistStamped, f'/oarbot_{r}/fabric_cbf/cmd_world', 10) for r in ROBOTS}
    latest = {}   # robot -> (wall time, frame, velocity, rtf)
    state = dict(last_print=0.0, received=0)

    def tick():
        now = time.monotonic()
        while True:
            try:
                msg = json.loads(sock.recv(4096))
            except BlockingIOError:
                break
            latest[msg['robot']] = (now, msg['frame'], msg['v'], msg['rtf'])
            state['received'] += 1
        for robot, (when, frame, v, rtf) in latest.items():
            if now - when > HOLD_SEC:
                continue
            scale = 1.0 if args.no_time_scale or rtf is None else min(max(rtf, 0.05), 1.5)
            out = TwistStamped()
            out.header.stamp = node.get_clock().now().to_msg()
            out.header.frame_id = frame
            out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = (c * scale for c in v)
            pubs[robot].publish(out)
        if now - state['last_print'] > 5.0:
            parts = []
            for robot in ROBOTS:
                if robot in latest and now - latest[robot][0] <= HOLD_SEC:
                    _, _, v, rtf = latest[robot]
                    speed = sum(c * c for c in v) ** 0.5
                    parts.append(f'{robot} {speed*100:.1f} cm/s sim' + (f' x{rtf:.2f}' if rtf else ''))
                else:
                    parts.append(f'{robot} silent')
            print(f'Relay (real side): {", ".join(parts)}; {state["received"]} commands so far', flush=True)
            state['last_print'] = now

    node.create_timer(0.02, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('side', choices=('sim', 'real'))
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--no-time-scale', action='store_true', help='real side: do not scale by the real-time factor')
    args = parser.parse_args()
    run_sim(args) if args.side == 'sim' else run_real(args)


if __name__ == '__main__':
    main()
