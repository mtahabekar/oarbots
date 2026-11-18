#!/usr/bin/env python3
"""Parking guide for the Dingo bases, from the overhead camera and the robots' roof markers.

`serve` runs on the NUC (the 1080p frames are local there): it finds roof markers 0 (Blue) and 4
(Silver) in each overhead frame, intersects the corner rays with the horizontal plane at the marker
height (more accurate than a single-marker PnP depth at 3-4 m), takes base_link 0.25 m ahead of the
marker centre along the marker's x axis (robot_arm_riser_aruco_mount_attach_link -> base_link in
the robot description), and writes an annotated view and the poses to /dev/shm every frame.
`view` runs on the laptop: it fetches the view over SSH and shows it in a window.

A marker that ArUco cannot find, because it is partly hidden (Silver's arm covers Blue's from the
camera at Home), is fitted instead: World.fit_marker matches the marker's known edges to the visible
part of the image (within 6 mm and 0.8 deg of the corner-based pose with up to 60 % of the marker
covered, 2026-11-18).

Green: target footprint and heading. Red: where the base is now (orange: from a fitted marker). Text: the move in the robot's own
frame (forward, left, turn) and OK when within 2 cm and 2 degrees.

    NUC:    python3 park_guide.py serve overhead_world.json        (domain 0, stock camera)
    NUC:    python3 park_guide.py measure overhead_world.json 10   (mean poses over 10 s, then exit)
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
MARKER_SIDE_M = 0.36         # roof markers' black square (lab docs; 0.350-0.366 m recovered on 2026-11-17)
# Partially hidden marker (fit_marker): an edge of the pattern counts as seen when the image steps from
# dark to light across it by at least (1 - FIT_EDGE_TOL) of the marker's contrast; the fit needs
# FIT_MIN_SEEN of the edges seen.
FIT_EDGE_TOL = 0.5
FIT_MIN_SEEN = 0.3


def marker_edges(marker_id, depth=0.25):
    """The marker's black/white edges as point pairs in its own plane (units of the black square,
    centre at 0): for every boundary between a black and a white cell of the 6 x 6 grid, and between
    the black border and the printed white margin, three positions along the edge, each with one point
    `depth` cell inside the black side and one inside the white side (at most 0.2 cell into the
    margin). The order of the pairs does not depend on `depth`.

    Column (u) runs from corner 0 to corner 1 (the robot's forward axis), row (v) from corner 1 to 2."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw = cv2.aruco.drawMarker if hasattr(cv2.aruco, 'drawMarker') else cv2.aruco.generateImageMarker
    cells = np.ones((8, 8), bool)                          # white margin ring around the 6 x 6 marker
    cells[1:7, 1:7] = draw(dictionary, marker_id, 6) > 127  # True = white
    black, white = [], []

    def uv(row, col):                                       # cell coordinates (margin ring = -1 / 6) to plane
        return (col - 1) / 6 - 0.5, (row - 1) / 6 - 0.5

    for r in range(8):
        for c in range(8):
            for dr, dc in ((0, 1), (1, 0)):
                r2, c2 = r + dr, c + dc
                if r2 > 7 or c2 > 7 or cells[r, c] == cells[r2, c2]:
                    continue
                if not (1 <= r <= 6 and 1 <= c <= 6) and not (1 <= r2 <= 6 and 1 <= c2 <= 6):
                    continue                                # both in the margin
                (rb, cb), (rw, cw) = ((r, c), (r2, c2)) if not cells[r, c] else ((r2, c2), (r, c))
                inside_w = depth if 1 <= rw <= 6 and 1 <= cw <= 6 else min(depth, 0.2)   # margin: stay close
                for t in (0.2, 0.5, 0.8):
                    # edge between the cells; offsets measured from it into each cell
                    if dr == 0:                             # vertical edge between columns
                        edge_col = max(cb, cw)
                        black.append(uv(rb + t, edge_col + (-depth if cb < cw else depth)))
                        white.append(uv(rw + t, edge_col + (inside_w if cw > cb else -inside_w)))
                    else:                                   # horizontal edge between rows
                        edge_row = max(rb, rw)
                        black.append(uv(edge_row + (-depth if rb < rw else depth), cb + t))
                        white.append(uv(edge_row + (inside_w if rw > rb else -inside_w), cw + t))
    return np.array(black), np.array(white)


class World:
    def __init__(self, path):
        w = json.load(open(path))
        self.K, self.D = np.array(w['K']), np.array(w['D'])
        self.O, self.R = np.array(w['origin_in_camera']), np.array(w['axes_in_camera_columns_xyz'])
        self.hm, self.ahead = w['marker_top_height_m'], w['marker_to_base_link_forward_m']
        self.ids = {v: k for k, v in w['robot_marker_ids'].items()}
        self.marker_ids = dict(w['robot_marker_ids'])
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

    def marker_pixels(self, uv, poses):
        """Image pixels of marker-plane points `uv` for base poses (x, y, yaw deg), shape (poses, points, 2)."""
        poses = np.atleast_2d(poses)
        yaw = np.radians(poses[:, 2])
        f = np.stack([np.cos(yaw), np.sin(yaw), np.zeros_like(yaw)], 1)          # corner 0 -> 1
        d = np.stack([np.sin(yaw), -np.cos(yaw), np.zeros_like(yaw)], 1)         # corner 1 -> 2 (seen from above)
        centre = np.c_[poses[:, :2], np.full(len(poses), self.hm)] - self.ahead * f
        pts = (centre[:, None, :] + MARKER_SIDE_M * (uv[None, :, :1] * f[:, None, :] + uv[None, :, 1:] * d[:, None, :]))
        cam = (self.O[None, None, :] + pts @ self.R.T).reshape(-1, 3)
        px = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), self.K, self.D)[0].reshape(len(poses), -1, 2)
        return px

    def _edge_steps(self, gray, pairs, grid):
        """Image step (white side minus black side) at every edge pair for every candidate pose."""
        h, w = gray.shape
        steps, inside = [], None
        for uv in pairs:
            px = self.marker_pixels(uv, grid)
            ok = (px[..., 0] >= 0) & (px[..., 0] < w - 1) & (px[..., 1] >= 0) & (px[..., 1] < h - 1)
            inside = ok if inside is None else inside & ok
            steps.append(cv2.remap(gray, np.clip(px[..., 0], 0, w - 1).astype(np.float32),
                                   np.clip(px[..., 1], 0, h - 1).astype(np.float32), cv2.INTER_LINEAR).astype(float))
        return steps[1] - steps[0], inside

    @staticmethod
    def _grid(best, span_m, step_m, span_deg, step_deg):
        offsets = np.arange(-span_m, span_m + 1e-9, step_m)
        turns = np.arange(-span_deg, span_deg + 1e-9, step_deg)
        return np.array([(best[0] + a, best[1] + b, best[2] + c) for a in offsets for b in offsets for c in turns])

    def fit_marker(self, gray, marker_id, init,
                   coarse=((0.08, 0.02, 8.0, 2.0), (0.02, 0.005, 2.0, 0.5)),
                   fine=((0.012, 0.003, 1.2, 0.3), (0.003, 0.0005, 0.3, 0.05))):
        """Base pose from a roof marker that may be partly hidden (by the other robot's arm, a cable).

        The marker lies flat at a known height under a calibrated camera, so only x, y and yaw are
        free. Each candidate pose projects the marker's black/white edges into the image and checks
        that the image steps from dark to light across each of them by the marker's contrast. An
        occluder of any shade has no such steps where the pattern expects them, so its edges count as
        unseen at a fixed cost and cannot pull the fit (a brightness match could: a dark occluder over
        black cells looks like a match).

        Coarse stages: points a quarter cell either side of each edge, robust to shifts up to that,
        grid search around `init`. They decide which edges are seen. Fine stages: only those edges,
        points 0.08 cell either side, where the image's blur makes the step change with sub-millimetre
        moves. Returns (x, y, yaw_deg, seen fraction of edges) or None when too little is seen."""
        far, near = marker_edges(marker_id, 0.25), marker_edges(marker_id, 0.08)
        best = np.array(init, float)
        for stage in coarse:
            grid = self._grid(best, *stage)
            step, inside = self._edge_steps(gray, far, grid)
            # The marker's contrast: the 80th percentile of the steps (needs >= 20 % of the edges seen).
            contrast = np.maximum(np.percentile(np.where(inside, step, 0), 80, axis=1), 1.0)
            e = step / contrast[:, None]
            cost = np.where(inside, np.minimum((e - 1) ** 2, FIT_EDGE_TOL ** 2), FIT_EDGE_TOL ** 2).mean(1)
            cost[contrast < 30] = np.inf                      # no black/white marker there
            k = int(np.argmin(cost))
            best = grid[k]
        if not np.isfinite(cost[k]):
            return None
        seen_edges = inside[k] & (np.abs(e[k] - 1) < FIT_EDGE_TOL)
        seen = float(np.mean(seen_edges))
        if seen < FIT_MIN_SEEN:
            return None
        near_seen = (near[0][seen_edges], near[1][seen_edges])
        for stage in fine:
            grid = self._grid(best, *stage)
            step, inside = self._edge_steps(gray, near_seen, grid)
            # Largest total dark-to-light step: the edges sit exactly between their sample points.
            k = int(np.argmax(np.where(inside, step, 0).mean(1)))
            best = grid[k]
        return best[0], best[1], best[2], seen


def footprint(x, y, yaw):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    f, l = np.array([c, s, 0]), np.array([-s, c, 0])
    b = np.array([x, y, 0])
    return [b + a * FOOT[0] * f + q * FOOT[1] * l for a, q in ((1, 1), (1, -1), (-1, -1), (-1, 1))], b, b + 0.45 * f


FIT_PERIOD_S = 0.5           # a partly hidden marker is re-fitted at most this often (a fit takes ~0.3 s)


def locate(world, gray, dictionary, state):
    """Base poses {robot: (x, y, yaw_deg, how)} from one overhead frame. A marker ArUco finds gives the
    pose from its corners; one it does not find (partly hidden, e.g. by the other robot's arm) is fitted
    (World.fit_marker), starting from the robot's last pose, or from its target with a wider search.
    `state` keeps the last poses and fit times between frames."""
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    poses = {}
    for c, i in zip(corners or [], [] if ids is None else ids.ravel()):
        if int(i) in world.ids:
            poses[world.ids[int(i)]] = (*world.base_from_corners(c[0]), 'marker')
    now = time.monotonic()
    for robot, target in world.targets.items():
        if robot in poses:
            state.setdefault('last', {})[robot] = poses[robot][:3]
            continue
        last = state.get('last', {}).get(robot)
        if last is not None and now - state.get('fit_time', {}).get(robot, -1e9) < FIT_PERIOD_S:
            held = state.get('fit', {}).get(robot)
            if held:
                poses[robot] = (*held[:3], f'fit {held[3]:.0%} seen, held')
            continue
        coarse = ((0.08, 0.02, 8.0, 2.0), (0.02, 0.005, 2.0, 0.5)) if last is not None else \
                 ((0.20, 0.04, 15.0, 3.0), (0.04, 0.01, 3.0, 1.0), (0.01, 0.005, 1.0, 0.5))
        fit = world.fit_marker(gray, world.marker_ids[robot], last if last is not None else target, coarse=coarse)
        state.setdefault('fit_time', {})[robot] = now
        state.setdefault('fit', {})[robot] = fit
        if fit:
            state.setdefault('last', {})[robot] = fit[:3]
            poses[robot] = (*fit[:3], f'fit {fit[3]:.0%} seen')
    return poses


def serve(world_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    world = World(world_path)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    state = {}

    def on_image(msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3].copy()
        found = locate(world, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), dictionary, state)
        img = annotate(world, img, found)
        poses = {robot: list(p[:3]) for robot, p in found.items()}
        cv2.imwrite('/dev/shm/park_tmp.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        os.replace('/dev/shm/park_tmp.jpg', '/dev/shm/park.jpg')
        with open('/dev/shm/park_tmp.json', 'w') as fh:
            json.dump({'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 'poses': poses,
                       'how': {robot: p[3] for robot, p in found.items()}}, fh)
        os.replace('/dev/shm/park_tmp.json', '/dev/shm/park.json')

    rclpy.init()
    node = Node('park_guide')
    node.create_subscription(Image, '/nuc/rgb/image_raw', on_image, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def annotate(world, img, found):
    """Targets (green), current footprints (red; orange when fitted to a partly hidden marker) and the
    moves to the targets, as a 1280 x 720 view."""
    lines = []
    for robot, (tx, ty, tyaw) in world.targets.items():
        name = robot.replace('oarbot_', '').capitalize()
        poly, b, h = footprint(tx, ty, tyaw)
        cv2.polylines(img, [world.project(poly)], True, (0, 220, 0), 4)
        p = world.project([b, h]); cv2.arrowedLine(img, tuple(p[0]), tuple(p[1]), (0, 220, 0), 4)
        cv2.putText(img, name + ' target', tuple(p[0] + [-60, 40]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 0), 2)
        if robot not in found:
            lines.append((f'{name}: marker not seen', (0, 165, 255))); continue
        x, y, yaw, how = found[robot]
        colour = (0, 0, 255) if how == 'marker' else (0, 140, 255)
        poly, b, h = footprint(x, y, yaw)
        cv2.polylines(img, [world.project(poly)], True, colour, 4)
        p = world.project([b, h]); cv2.arrowedLine(img, tuple(p[0]), tuple(p[1]), colour, 4)
        dx, dy = tx - x, ty - y
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        fwd, left = c * dx + s * dy, -s * dx + c * dy
        turn = (tyaw - yaw + 180) % 360 - 180
        ok = math.hypot(dx, dy) < TOL_M and abs(turn) < TOL_DEG
        lines.append((f'{name}: at ({x:+.2f}, {y:+.2f}) {yaw:+.0f} deg | forward {fwd:+.3f} m  left {left:+.3f} m  '
                      f'turn {"left" if turn > 0 else "right"} {abs(turn):.1f} deg' + ('   OK' if ok else '')
                      + ('' if how == 'marker' else f'  [{how}]'),
                      (0, 200, 0) if ok else (0, 0, 255)))
    small = cv2.resize(img, (1280, 720))
    cv2.rectangle(small, (0, 0), (1280, 34 * len(lines) + 12), (255, 255, 255), -1)
    for k, (text, col) in enumerate(lines):
        cv2.putText(small, text, (10, 32 + 34 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.75, col, 2)
    cv2.putText(small, time.strftime('%H:%M:%S'), (1150, 710), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return small


def measure(world_path, seconds=10.0):
    """Mean base poses over `seconds` of overhead frames (marker corners, or the fit when a marker is
    partly hidden), with their scatter and the move to each target; saves /dev/shm/park_measure.jpg."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    world = World(world_path)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    state, samples, last = {}, {}, {}

    def on_image(msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3].copy()
        found = locate(world, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), dictionary, state)
        for robot, p in found.items():
            if not p[3].endswith('held'):
                samples.setdefault(robot, []).append(p)
        last['img'] = annotate(world, img, found)

    rclpy.init()
    node = Node('park_measure')
    node.create_subscription(Image, '/nuc/rgb/image_raw', on_image, qos_profile_sensor_data)
    end = time.monotonic() + float(seconds)
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if 'img' in last:
        cv2.imwrite('/dev/shm/park_measure.jpg', last['img'], [cv2.IMWRITE_JPEG_QUALITY, 85])
    out = {}
    for robot, (tx, ty, tyaw) in world.targets.items():
        rows = samples.get(robot, [])
        if not rows:
            print(f'{robot}: not seen'); continue
        xyz = np.array([r[:3] for r in rows], float)
        m, sd = xyz.mean(0), xyz.std(0)
        hows = sorted({r[3].split(' ')[0] for r in rows})
        dx, dy = tx - m[0], ty - m[1]
        out[robot] = {'n': len(rows), 'how': hows, 'x': round(m[0], 4), 'y': round(m[1], 4), 'yaw_deg': round(m[2], 2),
                      'sd_mm': [round(1000 * sd[0], 1), round(1000 * sd[1], 1)], 'sd_yaw_deg': round(sd[2], 2),
                      'to_target_mm': round(1000 * math.hypot(dx, dy), 1), 'to_target_deg': round((tyaw - m[2] + 180) % 360 - 180, 2)}
        print(json.dumps({'robot': robot, **out[robot]}))
    rclpy.try_shutdown()
    return out


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
    modes = {'serve': serve, 'view': view, 'relay': relay, 'measure': measure}
    if len(sys.argv) < 3 or sys.argv[1] not in modes:
        sys.exit(__doc__)
    modes[sys.argv[1]](*sys.argv[2:])
