#!/usr/bin/env python3
"""Glove-tag check for one camera of the real cell: what the glove detector sees, and why not.

Runs on the computer the camera is plugged into (its 1080p frames do not cross the network), in the
fabric session:

    lab.sh session <host> exec python3 fabric_ws/scripts/hw/glove_check.py --camera overhead \
        --seconds 20 --out fabric_ws/run/glove_check

For each processed frame it runs the cell's `GloveDetector` with the camera's perception parameters
(`<cell_dir>/perception_<camera>.yaml`, or --params) exactly as `fabric_perception` does, and also a
full-frame ArUco search, so a tag that the detector misses shows the reason: no glove colour around
it (support), a rejected pose (reprojection or an ambiguity that depth did not settle), or a search
region that did not include it. It prints one line per frame, writes `frames.jsonl`, and saves the
raw colour and registered depth (`--save-every` seconds) with `camera_info.json`, for offline tuning
with the same code. Positions are the grip points in the camera's optical frame; for the overhead
camera also in the cell's world frame (2026-11-17 world definition).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from human_hand_tracking.tags import GloveDetector

NAMESPACES = {'blue_wrist': '/oarbot_blue/azure_kinect', 'silver_wrist': '/oarbot_silver/azure_kinect',
              'overhead': '/nuc'}
# world -> nuc/rgb_camera_link (optical), experiments/hardware/sessions/2026-11-17_world/overhead_extrinsics.json
OVERHEAD_WORLD = {'translation': [1.7049, -0.6535, 2.5971], 'rotation_xyzw': [0.666265, 0.66164, -0.249466, -0.236837]}


def rotation_matrix(x, y, z, w):
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def load_params(path):
    data = yaml.safe_load(Path(path).read_text())
    return next(iter(data.values()))['ros__parameters']


def detector_from(p):
    """The same construction as fabric_perception/node.py."""
    return GloveDetector(p.get('glove_tag_ids', [21, 20]), p.get('glove_tag_size_m', .04),
                         p.get('tag_to_grip_offsets_m', [0.] * 6), p.get('glove_hsv_lower', [35, 80, 60]),
                         p.get('glove_hsv_upper', [90, 255, 255]), p.get('glove_tag_dictionary', 'DICT_4X4_50'),
                         p.get('tag_max_reprojection_px', 2.), p.get('tag_min_color_support', .08),
                         p.get('tag_pixel_stddev', .7), p.get('tag_position_floor_m', .01),
                         p.get('glove_min_area_px', 100), p.get('tag_depth_gate_m', .005),
                         p.get('tag_max_range_correction', .25), p.get('tag_roi_upscale', 1),
                         p.get('tag_roi_fallback_unscaled', False))


def analyse(det, bgr, depth, k, dist, use_depth, world=None):
    """One frame: detector output plus a per-tag explanation from a full-frame search."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, det.lower, det.upper)
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    regions = int(np.sum(stats[1:, cv2.CC_STAT_AREA] >= det.min_area)) if count > 1 else 0
    _, hands = det.detect(bgr, k, dist, depth=depth if use_depth else None, hsv=hsv)
    found = {h.tag_id: h for h in hands}
    gated_corners, gated_ids = det.markers(bgr, mask)
    gated = set() if gated_ids is None else {int(i) for i in gated_ids.ravel()}
    corners, ids = det._aruco(bgr)
    tags = []
    for quad, tag in zip(corners, ids):
        pts = np.asarray(quad[0], np.float64)
        side = float(np.mean(np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1)))
        centre = pts.mean(axis=0)
        radius = max(2, int(np.linalg.norm(pts[0] - pts[2])))
        x, y = np.rint(centre).astype(int)
        y0, y1, x0, x1 = max(0, y-radius), y+radius+1, max(0, x-radius), x+radius+1
        crop = hsv[y0:y1, x0:x1]
        support = float(np.mean(cv2.inRange(crop, det.lower, det.upper) > 0))
        ring = np.full(crop.shape[:2], 255, np.uint8)
        cv2.fillConvexPoly(ring, np.rint(pts - [x0, y0]).astype(np.int32), 0)
        ring_hsv = crop[ring > 0]
        # The glove's own colour: a band from 1.15 to 1.6 tag sizes around the tag (outside a printed
        # white margin, mostly on the glove), as 10th/50th/90th percentiles of H, S and V.
        outer = np.zeros(hsv.shape[:2], np.uint8)
        cv2.fillConvexPoly(outer, np.rint(centre + 1.6 * (pts - centre)).astype(np.int32), 255)
        cv2.fillConvexPoly(outer, np.rint(centre + 1.15 * (pts - centre)).astype(np.int32), 0)
        band = hsv[outer > 0]
        entry = {'id': tag, 'side_px': round(side, 1), 'centre_px': [round(float(c), 1) for c in centre],
                 'support': round(support, 3), 'ring_hsv_median': [int(v) for v in np.median(ring_hsv, axis=0)],
                 'band_hsv_p10_50_90': np.percentile(band, [10, 50, 90], axis=0).round().astype(int).T.tolist()
                 if len(band) else None,
                 'in_search_region': tag in gated}
        if tag in det.identities:
            result = cv2.solvePnPGeneric(det.object_points, pts, k, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            errs = sorted(float(e) for e in np.ravel(result[3]))
            entry['reprojection_px'] = [round(e, 2) for e in errs]
            plain = det.from_corners(tag, pts, k, dist, support, None)
            with_depth = det.from_corners(tag, pts, k, dist, support, depth)
            entry['pose_without_depth'] = plain is not None
            entry['pose_with_depth'] = with_depth is not None
            if depth is not None:
                window = depth[max(0, y-2):y+3, max(0, x-2):x+3]
                window = window[np.isfinite(window) & (window > 0)]
                entry['depth_at_tag_m'] = round(float(np.median(window)), 3) if len(window) else None
        if tag in found:
            h = found[tag]
            entry['detected'] = True
            entry['grip_camera_m'] = [round(float(v), 3) for v in h.position]
            entry['sd_m'] = [round(float(v), 4) for v in np.sqrt(np.diag(h.covariance))]
            if world is not None:
                entry['grip_world_m'] = [round(float(v), 3) for v in world[0] @ h.position + world[1]]
        else:
            entry['detected'] = False
            if tag not in det.identities:
                entry['why'] = 'not a glove ID'
            elif support < det.min_support:
                entry['why'] = f'glove colour support {support:.2f} < {det.min_support}'
            elif tag not in gated:
                entry['why'] = 'outside the colour-gated search regions'
            elif not entry.get('pose_with_depth' if use_depth else 'pose_without_depth'):
                entry['why'] = 'pose rejected (reprojection, ambiguity or depth)'
            else:
                entry['why'] = 'other'
        tags.append(entry)
    return {'glove_mask_px': int(np.count_nonzero(mask)), 'glove_regions': regions,
            'detected': sorted(found), 'tags': tags}, mask, corners, ids


def annotate(bgr, mask, corners, ids, frame):
    view = bgr.copy()
    view[mask > 0] = (0.5 * view[mask > 0] + [0, 127, 0]).astype(np.uint8)
    by_id = {t['id']: t for t in frame['tags']}
    for quad, tag in zip(corners, ids):
        t = by_id.get(tag, {})
        colour = (0, 255, 0) if t.get('detected') else (0, 0, 255)
        cv2.polylines(view, [np.rint(quad[0]).astype(np.int32)], True, colour, 3)
        label = f"{tag} {'OK' if t.get('detected') else t.get('why', '')}"
        x, y = np.rint(quad[0][0]).astype(int)
        cv2.putText(view, label, (int(x), int(y) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)
    return view


class Check(Node):
    def __init__(self, args, params):
        super().__init__('glove_check')
        self.args, self.params = args, params
        self.det = detector_from(params)
        self.use_depth = bool(params.get('tag_use_depth', False))
        self.bridge = CvBridge()
        self.info = None
        self.world = None
        if args.camera == 'overhead':
            self.world = (rotation_matrix(*OVERHEAD_WORLD['rotation_xyzw']), np.array(OVERHEAD_WORLD['translation']))
        self.out = Path(args.out).expanduser() / f"{args.camera}_{time.strftime('%Y%m%d_%H%M%S')}"
        self.out.mkdir(parents=True, exist_ok=True)
        self.log = (self.out / 'frames.jsonl').open('w')
        (self.out / 'params.json').write_text(json.dumps(params, indent=1))
        self.last_processed = self.last_saved = 0.0
        self.frames = self.saved = 0
        self.counts = {}
        ns = NAMESPACES[args.camera]
        self.create_subscription(CameraInfo, ns + '/rgb/camera_info', self.on_info, qos_profile_sensor_data)
        sync = ApproximateTimeSynchronizer([Subscriber(self, Image, ns + '/rgb/image_raw', qos_profile=qos_profile_sensor_data),
                                            Subscriber(self, Image, ns + '/depth_to_rgb/image_raw',
                                                       qos_profile=qos_profile_sensor_data)], 5, 0.015)
        sync.registerCallback(self.on_images)
        self.start = time.monotonic()

    def on_info(self, msg):
        if self.info is None:
            (self.out / 'camera_info.json').write_text(json.dumps(
                {'width': msg.width, 'height': msg.height, 'k': list(msg.k), 'd': list(msg.d),
                 'distortion_model': msg.distortion_model, 'frame_id': msg.header.frame_id}))
        self.info = msg

    def on_images(self, rgb, depth_msg):
        now = time.monotonic()
        if self.info is None or now - self.last_processed < 1.0 / self.args.rate:
            return
        self.last_processed = now
        bgr = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        depth = raw_depth.astype(np.float32) * 0.001 if raw_depth.dtype == np.uint16 else raw_depth.astype(np.float32)
        k = np.array(self.info.k, np.float64).reshape(3, 3)
        dist = np.array(self.info.d, np.float64)
        t0 = time.monotonic()
        frame, mask, corners, ids = analyse(self.det, bgr, depth, k, dist, self.use_depth, self.world)
        frame['t'] = round(now - self.start, 2)
        frame['stamp'] = rgb.header.stamp.sec + rgb.header.stamp.nanosec * 1e-9
        frame['analysis_ms'] = round((time.monotonic() - t0) * 1000)
        self.frames += 1
        for tag in frame['detected']:
            self.counts[tag] = self.counts.get(tag, 0) + 1
        if self.args.save_every > 0 and now - self.last_saved >= self.args.save_every:
            self.last_saved = now
            name = f'frame_{self.saved:03d}'
            cv2.imwrite(str(self.out / f'{name}_color.png'), bgr)
            if raw_depth.dtype == np.uint16:
                cv2.imwrite(str(self.out / f'{name}_depth.png'), raw_depth)
            view = annotate(bgr, mask, corners, ids, frame)
            cv2.imwrite(str(self.out / f'{name}_annotated.jpg'), cv2.resize(view, None, fx=0.5, fy=0.5),
                        [cv2.IMWRITE_JPEG_QUALITY, 85])
            frame['saved'] = name
            self.saved += 1
        self.log.write(json.dumps(frame) + '\n')
        self.log.flush()
        tags = '; '.join(
            f"{t['id']} {'OK' if t['detected'] else 'missed: ' + t.get('why', '')} side {t['side_px']} px"
            f" support {t['support']:.2f} around tag (p10,p50,p90) "
            + (' '.join(f'{c}{v}' for c, v in zip('HSV', t['band_hsv_p10_50_90'])) if t['band_hsv_p10_50_90'] else '-')
            + (f" grip world {t['grip_world_m']}" if 'grip_world_m' in t else
               f" grip cam {t['grip_camera_m']}" if 'grip_camera_m' in t else '')
            for t in frame['tags']) or 'no tags in frame'
        print(f"[{frame['t']:6.1f}s] glove px {frame['glove_mask_px']:7d} regions {frame['glove_regions']:2d} | {tags}",
              flush=True)

    def done(self):
        return time.monotonic() - self.start >= self.args.seconds


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', choices=sorted(NAMESPACES), required=True)
    parser.add_argument('--cell-dir', default=str(Path(__file__).resolve().parents[2] / 'experiments/hardware/cell'))
    parser.add_argument('--params', help='perception YAML to use instead of <cell_dir>/perception_<camera>.yaml')
    parser.add_argument('--seconds', type=float, default=20.0)
    parser.add_argument('--rate', type=float, default=3.0, help='frames analysed per second (full-frame ArUco is slow)')
    parser.add_argument('--save-every', type=float, default=2.0, help='seconds between saved frames; 0 saves none')
    parser.add_argument('--out', default='~/fabric_ws/run/glove_check')
    args = parser.parse_args()
    params = load_params(args.params or Path(args.cell_dir) / f'perception_{args.camera}.yaml')
    rclpy.init()
    node = Check(args, params)
    print(f'{args.camera}: tags {params.get("glove_tag_ids")} {params.get("glove_tag_size_m")} m, HSV '
          f'{params.get("glove_hsv_lower")}..{params.get("glove_hsv_upper")}, support >= {params.get("tag_min_color_support")},'
          f' depth {"on" if node.use_depth else "off"}; output {node.out}', flush=True)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        summary = {'frames': node.frames, 'detections': node.counts, 'saved': node.saved, 'out': str(node.out)}
        (node.out / 'summary.json').write_text(json.dumps(summary, indent=1))
        print('summary:', json.dumps(summary), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if node.frames else 1


if __name__ == '__main__':
    sys.exit(main())
