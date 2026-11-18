#!/usr/bin/env python3
"""Record what one camera's fabric_perception node reports about the gloves (runs anywhere on the session
domain: the glove messages are small), and summarise it.

    source scripts/hw/laptop_env.sh
    python3 scripts/hw/glove_record.py --camera overhead --seconds 90 --out experiments/hardware/sessions/.../live.jsonl
    python3 scripts/hw/glove_record.py --score live.jsonl        # score a recording (no ROS needed)

Per message: capture stamp, arrival age, and each hand's id, tag, grip position (camera frame; world
frame for the overhead camera, 2026-11-17 world) and position sd. The summary splits the run into
still and moving stretches by hand speed (below 3 cm/s over 0.5 s counts as still), so the operator's
timing need not be exact, and reports per hand: detection rate, longest gap, position scatter while
still, and the camera's own diagnostics (processing time, capture age, drop reasons).
"""
import argparse
import json
import time

import numpy as np

try:  # recording needs ROS and the fabric workspace; scoring a recording (--score) does not
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from fabric_state_msgs.msg import GloveObservations
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
except ImportError:
    rclpy, Node = None, object

NAMESPACES = {'blue_wrist': '/oarbot_blue/azure_kinect', 'silver_wrist': '/oarbot_silver/azure_kinect', 'overhead': '/nuc'}
OVERHEAD_WORLD = {'translation': [1.7049, -0.6535, 2.5971], 'rotation_xyzw': [0.666265, 0.66164, -0.249466, -0.236837]}
STILL_MPS = 0.03


def rotation(x, y, z, w):
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


class Recorder(Node):
    def __init__(self, camera, out):
        super().__init__('glove_record')
        ns = NAMESPACES[camera]
        self.world = None
        if camera == 'overhead':
            self.world = (rotation(*OVERHEAD_WORLD['rotation_xyzw']), np.array(OVERHEAD_WORLD['translation']))
        self.out = open(out, 'w') if out else None
        self.rows, self.diag = [], []
        self.create_subscription(GloveObservations, ns + '/fabric/gloves', self.on_gloves, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, ns + '/fabric/diagnostics', self.on_diag, 10)

    def on_gloves(self, msg):
        now = time.time()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        hands = []
        for h in msg.hands:
            p = np.array([h.position.x, h.position.y, h.position.z])
            entry = {'hand': h.hand_id, 'tag': h.tag_id, 'camera_m': p.round(4).tolist(),
                     'sd_m': np.sqrt(np.diag(np.array(h.covariance).reshape(3, 3))).round(4).tolist(),
                     'reprojection_px': round(h.reprojection_error_px, 2), 'support': round(h.color_support, 3)}
            if self.world is not None:
                entry['world_m'] = (self.world[0] @ p + self.world[1]).round(4).tolist()
            hands.append(entry)
        row = {'stamp': stamp, 'age_s': round(now - stamp, 3), 'hands': hands}
        self.rows.append(row)
        if self.out:
            self.out.write(json.dumps(row) + '\n')

    def on_diag(self, msg):
        for s in msg.status:
            self.diag.append({'level': int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level),
                              'message': s.message, **{v.key: v.value for v in s.values}})


def summarise(rows, diag):
    if not rows:
        return {'messages': 0}
    t = np.array([r['stamp'] for r in rows])
    out = {'messages': len(rows), 'duration_s': round(float(t[-1] - t[0]), 1),
           'rate_hz': round(len(rows) / max(1e-6, float(t[-1] - t[0])), 2),
           'age_s_median_p95': [round(float(np.median([r['age_s'] for r in rows])), 3),
                                round(float(np.percentile([r['age_s'] for r in rows], 95)), 3)]}
    key = 'world_m' if any('world_m' in h for r in rows for h in r['hands']) else 'camera_m'
    for hand in ('human_left', 'human_right'):
        seen = [(r['stamp'], np.array(next(h[key] for h in r['hands'] if h['hand'] == hand)),
                 next(h['tag'] for h in r['hands'] if h['hand'] == hand))
                for r in rows if any(h['hand'] == hand for h in r['hands'])]
        if not seen:
            out[hand] = {'detected_in': f'0/{len(rows)}'}
            continue
        ts = np.array([s[0] for s in seen]); ps = np.array([s[1] for s in seen])
        gaps = np.diff(np.r_[t[0], ts, t[-1]])
        # speed over ~0.5 s windows: still where it stays below STILL_MPS
        speed = np.full(len(ts), np.nan)
        for i in range(len(ts)):
            j = np.searchsorted(ts, ts[i] - 0.5)
            if ts[i] - ts[j] > 0.3:
                speed[i] = np.linalg.norm(ps[i] - ps[j]) / (ts[i] - ts[j])
        still = speed < STILL_MPS
        info = {'tag': sorted({s[2] for s in seen}), 'detected_in': f'{len(seen)}/{len(rows)} ({100*len(seen)/len(rows):.0f} %)',
                'longest_gap_s': round(float(gaps.max()), 2), 'mean_position_m': ps.mean(0).round(3).tolist(),
                'still_samples': int(np.sum(still)), 'moving_samples': int(np.sum(speed >= STILL_MPS))}
        if np.sum(still) > 5:
            info['still_sd_mm'] = (1000 * ps[still].std(0)).round(1).tolist()
        out[hand] = info
    if diag:
        def values(k):
            return [float(d[k]) for d in diag if k in d and d[k] not in ('', 'None')]
        out['camera_diagnostics'] = {
            'processing_ms_median_p95': [round(float(np.median(values('processing_ms'))), 1),
                                         round(float(np.percentile(values('processing_ms'), 95)), 1)] if values('processing_ms') else None,
            'capture_age_ms_median': round(float(np.median(values('capture_age_ms'))), 1) if values('capture_age_ms') else None,
            'messages': sorted({d['message'] for d in diag})[:6]}
    return out


def score(path):
    """Score a recording over the seconds in which any glove was seen (the person may walk between
    cameras): per hand, detection share, gaps over 0.5 s, range, jitter about a 1 s running median (it
    also contains the hands' own motion) and the reported sd; and the share with both hands."""
    rows = [json.loads(line) for line in open(path)]
    key = 'world_m' if any('world_m' in h for r in rows for h in r['hands']) else 'camera_m'
    t = np.array([r['stamp'] for r in rows])
    t0 = t[0]
    sec = np.floor(t - t0).astype(int)
    seen = np.array([bool(r['hands']) for r in rows])
    view = np.isin(sec, sorted(set(sec[seen])))
    ages = [r['age_s'] for r in rows]
    out = {'messages': len(rows), 'rate_hz': round(len(rows) / max(1e-6, t[-1] - t0), 1),
           'age_s_median_p95': [round(float(np.median(ages)), 3), round(float(np.percentile(ages, 95)), 3)],
           'in_view_s': len(set(sec[seen])), 'in_view_messages': int(view.sum())}
    if not view.any():
        return out
    for hand in ('human_left', 'human_right'):
        has = np.array([any(h['hand'] == hand for h in r['hands']) for r in rows])
        pts = [(r['stamp'], np.array(next(h[key] for h in r['hands'] if h['hand'] == hand)),
                next(h['sd_m'] for h in r['hands'] if h['hand'] == hand)) for r in rows
               if any(h['hand'] == hand for h in r['hands'])]
        info = {'detected': f'{int(has[view].sum())}/{int(view.sum())} ({100 * has[view].mean():.0f} %)'}
        if pts:
            ts = np.array([p[0] for p in pts]); ps = np.array([p[1] for p in pts]); sds = np.array([p[2] for p in pts])
            gaps = np.diff(np.r_[t0 + sec[view].min(), ts, t0 + sec[view].max() + 1])
            med = np.array([np.median(ps[(ts >= s - 0.5) & (ts <= s + 0.5)], axis=0) for s in ts])
            info.update({'gaps_over_0.5s': int((gaps > 0.5).sum()), 'longest_gap_s': round(float(gaps.max()), 2),
                         'mean_position_m': ps.mean(0).round(3).tolist(),
                         'jitter_mm': (1000 * (ps - med).std(0)).round(1).tolist(),
                         'reported_sd_mm_median': (1000 * np.median(sds, 0)).round(1).tolist()})
        out[hand] = info
    both = np.array([len(r['hands']) == 2 for r in rows])
    out['both_hands'] = f'{100 * both[view].mean():.0f} %'
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', choices=sorted(NAMESPACES))
    parser.add_argument('--score', metavar='JSONL', help='score an existing recording and exit')
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--out', help='JSONL of every glove message')
    args = parser.parse_args()
    if args.score:
        print(json.dumps(score(args.score), indent=1))
        return
    if not args.camera:
        parser.error('--camera is required to record')
    if rclpy is None:
        parser.error('recording needs ROS and the fabric workspace sourced')
    rclpy.init()
    node = Recorder(args.camera, args.out)
    end = time.monotonic() + args.seconds
    print(f'recording {args.camera} gloves for {args.seconds:.0f} s', flush=True)
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    summary = summarise(node.rows, node.diag)
    print(json.dumps(summary, indent=1))
    if args.out:
        with open(args.out.replace('.jsonl', '_summary.json'), 'w') as fh:
            json.dump(summary, fh, indent=1)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
