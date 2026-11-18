#!/usr/bin/env python3
"""Parking guide for the Dingo bases, from the overhead camera and the robots' roof markers.

`serve` runs on the NUC (the 1080p frames are local there): it finds roof markers 0 (Blue) and 4
(Silver) in each overhead frame, intersects the corner rays with the horizontal plane at the marker
height (more accurate than a single-marker PnP depth at 3-4 m), takes base_link 0.25 m ahead of the
marker centre along the marker's x axis (robot_arm_riser_aruco_mount_attach_link -> base_link in
the robot description), and writes an annotated view and the poses to /dev/shm every frame.
`view` runs on the laptop: it fetches the view over SSH and shows it in a window.

Green: target footprint and heading. Red: where the base is now. Text: the move in the robot's own
frame (forward, left, turn) and OK when within 2 cm and 2 degrees.

    NUC:    python3 park_guide.py serve overhead_world.json        (domain 0, stock camera)
    NUC:    python3 park_guide.py relay -                          (session domain: poses for park_drive)
    laptop: python3 scripts/hw/park_guide.py view nuc@192.168.1.90
"""
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import cv2

FOOT = (0.34, 0.29)          # Dingo half length, half width (m)
TOL_M, TOL_DEG = 0.02, 2.0
NAMES = {0: 'Blue', 4: 'Silver'}


class World:
    def __init__(self, path):
        w = json.load(open(path))
        self.K, self.D = np.array(w['K']), np.array(w['D'])
        self.O, self.R = np.array(w['origin_in_camera']), np.array(w['axes_in_camera_columns_xyz'])
        self.hm, self.ahead = w['marker_top_height_m'], w['marker_to_base_link_forward_m']
        self.ids = {v: k for k, v in w['robot_marker_ids'].items()}
        self.targets = w['targets']
        self.ow = self.R.T @ (-self.O)                     # camera centre in world

    def project(self, Pw):
        Pc = (self.O[:, None] + self.R @ np.asarray(Pw, float).T).T
        return cv2.projectPoints(Pc, np.zeros(3), np.zeros(3), self.K, self.D)[0].reshape(-1, 2).astype(int)

    def base_from_corners(self, c):
        rays = cv2.undistortPoints(c.reshape(-1, 1, 2).astype(np.float64), self.K, self.D).reshape(-1, 2)
        pts = []
        for r in np.c_[rays, np.ones(4)]:
            rw = self.R.T @ r
            pts.append(self.ow + (self.hm - self.ow[2]) / rw[2] * rw)
        pts = np.array(pts)
        xm = (pts[1] - pts[0]) + (pts[2] - pts[3]); xm[2] = 0; xm /= np.linalg.norm(xm)
        base = pts.mean(0) + self.ahead * xm
        return base[0], base[1], math.degrees(math.atan2(xm[1], xm[0]))


def footprint(x, y, yaw):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    f, l = np.array([c, s, 0]), np.array([-s, c, 0])
    b = np.array([x, y, 0])
    return [b + a * FOOT[0] * f + q * FOOT[1] * l for a, q in ((1, 1), (1, -1), (-1, -1), (-1, 1))], b, b + 0.45 * f


def serve(world_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    world = World(world_path)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    def on_image(msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3].copy()
        corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), dictionary)
        found = {}
        for c, i in zip(corners or [], [] if ids is None else ids.ravel()):
            if int(i) in world.ids:
                found[world.ids[int(i)]] = world.base_from_corners(c[0])
        lines, poses = [], {}
        for robot, (tx, ty, tyaw) in world.targets.items():
            name = robot.replace('oarbot_', '').capitalize()
            poly, b, h = footprint(tx, ty, tyaw)
            cv2.polylines(img, [world.project(poly)], True, (0, 220, 0), 4)
            p = world.project([b, h]); cv2.arrowedLine(img, tuple(p[0]), tuple(p[1]), (0, 220, 0), 4)
            cv2.putText(img, name + ' target', tuple(p[0] + [-60, 40]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 0), 2)
            if robot not in found:
                lines.append((f'{name}: marker not seen', (0, 165, 255))); continue
            x, y, yaw = found[robot]; poses[robot] = [x, y, yaw]
            poly, b, h = footprint(x, y, yaw)
            cv2.polylines(img, [world.project(poly)], True, (0, 0, 255), 4)
            p = world.project([b, h]); cv2.arrowedLine(img, tuple(p[0]), tuple(p[1]), (0, 0, 255), 4)
            dx, dy = tx - x, ty - y
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            fwd, left = c * dx + s * dy, -s * dx + c * dy
            turn = (tyaw - yaw + 180) % 360 - 180
            ok = math.hypot(dx, dy) < TOL_M and abs(turn) < TOL_DEG
            lines.append((f'{name}: at ({x:+.2f}, {y:+.2f}) {yaw:+.0f} deg | forward {fwd:+.3f} m  left {left:+.3f} m  '
                          f'turn {"left" if turn > 0 else "right"} {abs(turn):.1f} deg' + ('   OK' if ok else ''),
                          (0, 200, 0) if ok else (0, 0, 255)))
        small = cv2.resize(img, (1280, 720))
        cv2.rectangle(small, (0, 0), (1280, 34 * len(lines) + 12), (255, 255, 255), -1)
        for k, (text, col) in enumerate(lines):
            cv2.putText(small, text, (10, 32 + 34 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.75, col, 2)
        cv2.putText(small, time.strftime('%H:%M:%S'), (1150, 710), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imwrite('/dev/shm/park_tmp.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 80])
        os.replace('/dev/shm/park_tmp.jpg', '/dev/shm/park.jpg')
        with open('/dev/shm/park_tmp.json', 'w') as fh:
            json.dump({'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 'poses': poses}, fh)
        os.replace('/dev/shm/park_tmp.json', '/dev/shm/park.json')

    rclpy.init()
    node = Node('park_guide')
    node.create_subscription(Image, '/nuc/rgb/image_raw', on_image, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def relay(_):
    """Publish /dev/shm/park.json on /park/poses (std_msgs/String, 10 Hz) for park_drive elsewhere."""
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('park_pose_relay')
    pub = node.create_publisher(String, '/park/poses', 10)

    def tick():
        try:
            with open('/dev/shm/park.json') as fh:
                pub.publish(String(data=fh.read()))
        except OSError:
            pass
    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def view(host):
    ssh = os.environ.get('LAB_SSH', 'ssh').split()
    cv2.namedWindow('park guide', cv2.WINDOW_NORMAL)
    while True:
        try:
            data = subprocess.run(ssh + [host, 'cat /dev/shm/park.jpg'], capture_output=True, timeout=10).stdout
        except subprocess.TimeoutExpired:
            data = b''                                     # a slow fetch (busy NUC or network): keep the last view
        if data:
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow('park guide', img)
        if cv2.waitKey(300) in (27, ord('q')):
            break


if __name__ == '__main__':
    modes = {'serve': serve, 'view': view, 'relay': relay}
    if len(sys.argv) < 3 or sys.argv[1] not in modes:
        sys.exit(__doc__)
    modes[sys.argv[1]](sys.argv[2])
