#!/usr/bin/env python3
"""Offline glove calibration on frames saved by glove_check.py (runs on the laptop, no ROS graph).

Steps, as done for the overhead camera on 2026-11-18 (experiments/hardware/sessions/2026-11-18_glove_test):

1. Colour. Which hues sit next to the glove tags but rarely elsewhere in the image:
       glove_tune.py colour RUN_DIR
   then the saturation/brightness of a chosen hue band, and what else in the scene falls into a
   candidate range (region by region):
       glove_tune.py colour RUN_DIR --hue 85 105 --range 85 70 35 105 255 255
   and the colour of each look-alike, in boxes you pick on one frame:
       glove_tune.py sample RUN_DIR frame_033 --range 85 70 35 105 255 255 --box glove 1220 353 1340 406 ...
2. Evaluate a parameter set with the real GloveDetector on every saved frame: detections per tag,
   colour support, colour regions, timing, and for each tag the depth residual of both candidate poses
   (the pose ambiguity of small tags):
       glove_tune.py evaluate RUN_DIR --params perception_overhead.yaml --set tag_depth_gate_m=0.012
   --images DIR writes annotated frames (colour mask in magenta, colour regions boxed, tags green when
   detected with their world position, red with the reason when not).

RUN_DIR is one glove_check.py output folder: frames.jsonl, camera_info.json, frame_NNN_color.png and
frame_NNN_depth.png. Positions in world coordinates need --world (overhead: built in).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/human_hand_tracking'))
import glove_check as gc  # noqa: E402

GLOVE_IDS = (20, 21)


def saved_frames(run):
    """(name, frame record, bgr, depth in m) for every saved frame that shows a glove tag."""
    out = []
    for line in open(run / 'frames.jsonl'):
        f = json.loads(line)
        name = f.get('saved')
        if not name or not set(GLOVE_IDS) & {t['id'] for t in f['tags']}:
            continue
        bgr = cv2.imread(str(run / f'{name}_color.png'))
        raw = cv2.imread(str(run / f'{name}_depth.png'), cv2.IMREAD_UNCHANGED)
        if bgr is None or raw is None:
            continue
        out.append((name, f, bgr, raw.astype(np.float32) * 0.001))
    return out


def near_mask(shape, tags, reach=3.5, margin=1.45):
    """Pixels within `reach` tag sizes of a glove tag, minus the tag and its printed margin."""
    mask = np.zeros(shape, np.uint8)
    for t in tags:
        x, y = map(int, t['centre_px'])
        cv2.circle(mask, (x, y), int(reach * t['side_px']), 255, -1)
    for t in tags:
        x, y = map(int, t['centre_px'])
        s = int(0.5 * margin * t['side_px'] * 2)
        cv2.rectangle(mask, (x - s, y - s), (x + s, y + s), 0, -1)
    return mask


def regions(mask, min_area):
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    return [tuple(int(v) for v in s) for s in stats[1:] if s[4] >= min_area]


def cmd_colour(args):
    run = Path(args.run)
    frames = saved_frames(run)
    near, rest = [], []
    for _, f, bgr, _ in frames:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m = near_mask(hsv.shape[:2], [t for t in f['tags'] if t['id'] in GLOVE_IDS])
        coloured = (hsv[..., 1] >= args.min_s) & (hsv[..., 2] >= args.min_v)
        near.append(hsv[(m > 0) & coloured])
        rest.append(hsv[(m == 0) & coloured])
    near, rest = np.concatenate(near), np.concatenate(rest)
    hn = np.bincount(near[:, 0], minlength=180) / max(1, len(near))
    hr = np.bincount(rest[:, 0], minlength=180) / max(1, len(rest))
    print(f'{len(frames)} frames; coloured pixels (S >= {args.min_s}, V >= {args.min_v}): {len(near)} near the glove tags, '
          f'{len(rest)} elsewhere')
    print('hue  near%  elsewhere%  enrichment   (hue bins of 2, only bins with >= 0.5 % of the near pixels)')
    result = {'frames': len(frames), 'hue_table': []}
    for h in range(0, 180, 2):
        a, b = hn[h:h + 2].sum(), hr[h:h + 2].sum()
        if a >= 0.005:
            ratio = a / b if b > 0 else float('inf')
            print(f'{h:3d}  {100 * a:5.1f}  {100 * b:9.2f}  {ratio:9.1f}')
            result['hue_table'].append([h, round(100 * a, 2), round(100 * b, 3), round(ratio, 1)])
    if args.hue:
        lo, hi = args.hue
        band = near[(near[:, 0] >= lo) & (near[:, 0] <= hi)]
        pct = {c: np.percentile(band[:, i], [1, 5, 25, 50, 75, 95, 99]).round().astype(int).tolist()
               for i, c in enumerate('HSV')}
        print(f'near-tag pixels with H {lo}-{hi}: {len(band)}; percentiles 1/5/25/50/75/95/99: ' +
              '; '.join(f'{c} {v}' for c, v in pct.items()))
        result['band'] = {'hue': [lo, hi], 'pixels': int(len(band)), 'percentiles_1_5_25_50_75_95_99': pct}
    if args.range:
        lower, upper = np.array(args.range[:3]), np.array(args.range[3:])
        counts, far_counts, glove_frac = [], [], []
        for _, f, bgr, _ in frames:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            tags = [t for t in f['tags'] if t['id'] in GLOVE_IDS]
            regs = regions(mask, args.min_area)
            far = [r for r in regs if all(np.hypot(r[0] + r[2] / 2 - t['centre_px'][0], r[1] + r[3] / 2 - t['centre_px'][1])
                                          > 6 * t['side_px'] for t in tags)]
            counts.append(len(regs)), far_counts.append(len(far))
            m = near_mask(hsv.shape[:2], tags)
            glove_frac.append(np.count_nonzero(mask[m > 0]) / max(1, np.count_nonzero(m)))
        print(f'range {lower.tolist()}..{upper.tolist()}: colour regions >= {args.min_area} px per frame median/max '
              f'{int(np.median(counts))}/{max(counts)} (the detector searches the whole image above 8); regions away from '
              f'the gloves median {int(np.median(far_counts))}; share of near-tag area in range {np.median(glove_frac):.2f}')
        result['range'] = {'lower': lower.tolist(), 'upper': upper.tolist(), 'regions_median': int(np.median(counts)),
                           'regions_max': max(counts), 'regions_away_median': int(np.median(far_counts))}
    if args.json:
        Path(args.json).write_text(json.dumps(result, indent=1))


def cmd_sample(args):
    """HSV percentiles of the in-range pixels inside named boxes of one frame (glove vs look-alikes)."""
    run = Path(args.run)
    hsv = cv2.cvtColor(cv2.imread(str(run / f'{args.frame}_color.png')), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(args.range[:3]), np.array(args.range[3:]))
    result = {}
    for name, x0, y0, x1, y1 in args.box:
        x0, y0, x1, y1 = map(int, (x0, y0, x1, y1))
        px = hsv[y0:y1, x0:x1][mask[y0:y1, x0:x1] > 0]
        if not len(px):
            print(f'{name:16s} no pixels in range')
            continue
        p = np.percentile(px, [5, 25, 50, 75, 95], axis=0).round().astype(int)
        result[name] = {'box': [x0, y0, x1, y1], 'pixels': int(len(px)), 'H': p[:, 0].tolist(), 'S': p[:, 1].tolist(), 'V': p[:, 2].tolist()}
        print(f'{name:16s} n={len(px):5d}  percentiles 5/25/50/75/95  H {p[:, 0].tolist()}  S {p[:, 1].tolist()}  V {p[:, 2].tolist()}')
    if args.json:
        Path(args.json).write_text(json.dumps({'frame': args.frame, 'range': args.range, 'boxes': result}, indent=1))


def pose_diagnosis(det, tag, pts, k, dist, depth):
    """Both IPPE candidates: reprojection error and depth residual (the ambiguity of small tags)."""
    res = cv2.solvePnPGeneric(det.object_points, pts, k, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    cands = []
    for rvec, tvec in zip(res[1], res[2]):
        proj = cv2.projectPoints(det.object_points, rvec, tvec, k, dist)[0][:, 0]
        err = float(np.sqrt(np.mean(np.sum((proj - pts) ** 2, axis=1))))
        grip = cv2.Rodrigues(rvec)[0] @ det.offsets[tag] + tvec.ravel()
        cands.append((err, grip, det.depth_fit(rvec, tvec, pts, depth)[0]))
    cands.sort(key=lambda c: c[0])
    ambiguous = len(cands) > 1 and cands[1][0] < max(0.5, cands[0][0] * 1.5) and np.linalg.norm(cands[1][1] - cands[0][1]) > 0.03
    return {'reprojection_px': [round(c[0], 2) for c in cands], 'ambiguous': bool(ambiguous),
            'depth_residual_mm': [round(1000 * c[2], 1) for c in cands]}


def render(bgr, mask, regs, frame, title):
    view = bgr.copy()
    view[mask > 0] = (0.35 * view[mask > 0] + [165, 0, 165]).astype(np.uint8)
    for x, y, w, h, _ in regs:
        cv2.rectangle(view, (x - 4, y - 4), (x + w + 4, y + h + 4), (0, 165, 255), 2)
    for t in frame['tags']:
        if t['id'] not in GLOVE_IDS:
            continue
        x, y = map(int, t['centre_px'])
        s = int(t['side_px'])
        ok = t['detected']
        colour = (0, 220, 0) if ok else (0, 0, 255)
        cv2.rectangle(view, (x - s, y - s), (x + s, y + s), colour, 3)
        label = (f"{t['id']} OK  world {t['grip_world_m']}" if ok and 'grip_world_m' in t else
                 f"{t['id']} OK" if ok else f"{t['id']} missed: {t.get('why')}")
        cv2.putText(view, label, (x - s, y - s - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 5)
        cv2.putText(view, label, (x - s, y - s - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)
    cv2.putText(view, title, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 6)
    cv2.putText(view, title, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
    return view


def cmd_evaluate(args):
    run = Path(args.run)
    params = gc.load_params(args.params)
    for item in args.set or []:
        key, value = item.split('=', 1)
        params[key] = yaml.safe_load(value)
    det = gc.detector_from(params)
    use_depth = bool(params.get('tag_use_depth', False))
    info = json.loads((run / 'camera_info.json').read_text())
    k, dist = np.array(info['k'], np.float64).reshape(3, 3), np.array(info['d'], np.float64)
    world = None
    if args.world or 'overhead' in run.name:
        world = (gc.rotation_matrix(*gc.OVERHEAD_WORLD['rotation_xyzw']), np.array(gc.OVERHEAD_WORLD['translation']))
    out = Path(args.images) if args.images else None
    if out:
        out.mkdir(parents=True, exist_ok=True)
    seen, hit, why, support, regs_all, timing, residual_right, residual_wrong, per_frame = {}, {}, {}, {}, [], [], [], [], []
    reprojection = []
    for name, _, bgr, depth in saved_frames(run):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        t0 = time.monotonic()
        det.detect(bgr, k, dist, depth=depth if use_depth else None, hsv=hsv)
        timing.append(1000 * (time.monotonic() - t0))
        frame, mask, _, _ = gc.analyse(det, bgr, depth, k, dist, use_depth, world)
        regs = regions(mask, det.min_area)
        regs_all.append(len(regs))
        # Pose diagnosis on the corners the detector itself uses (its colour-gated search), which
        # depend on tag_roi_upscale; the full-frame search in analyse() is only for explaining misses.
        corners, ids = det.markers(bgr, mask)
        for quad, tag in zip(corners, [] if ids is None else [int(i) for i in ids.ravel()]):
            if tag in GLOVE_IDS:
                d = pose_diagnosis(det, tag, np.asarray(quad[0], np.float64), k, dist, depth)
                reprojection.append(d['reprojection_px'][0])
                if d['ambiguous'] and len(d['depth_residual_mm']) == 2:
                    residual_right.append(min(d['depth_residual_mm'])), residual_wrong.append(max(d['depth_residual_mm']))
        for t in frame['tags']:
            if t['id'] not in GLOVE_IDS:
                continue
            seen[t['id']] = seen.get(t['id'], 0) + 1
            support.setdefault(t['id'], []).append(t['support'])
            if t['detected']:
                hit[t['id']] = hit.get(t['id'], 0) + 1
            else:
                why[t.get('why')] = why.get(t.get('why'), 0) + 1
        per_frame.append({'frame': name, 'regions': len(regs), 'tags': frame['tags']})
        if out:
            view = render(bgr, mask, regs, frame, args.title or Path(args.params).name)
            cv2.imwrite(str(out / f'{name}.jpg'), cv2.resize(view, None, fx=args.scale, fy=args.scale),
                        [cv2.IMWRITE_JPEG_QUALITY, 85])
    summary = {
        'params': {k_: params[k_] for k_ in ('glove_hsv_lower', 'glove_hsv_upper', 'tag_min_color_support', 'tag_roi_upscale', 'tag_roi_fallback_unscaled',
                                            'tag_use_depth') if k_ in params} | {'tag_depth_gate_m': det.depth_gate},
        'detected': {str(t): f'{hit.get(t, 0)}/{seen[t]}' for t in sorted(seen)},
        'misses': why,
        'support_min_median': {str(t): [round(min(v), 3), round(float(np.median(v)), 3)] for t, v in support.items()},
        'colour_regions_median_max': [int(np.median(regs_all)), max(regs_all)] if regs_all else None,
        'detect_ms_median_max': [round(float(np.median(timing)), 1), round(max(timing), 1)] if timing else None,
        'search_reprojection_px_median_max': [round(float(np.median(reprojection)), 2), round(max(reprojection), 2)]
        if reprojection else None,
        'ambiguous_tags': len(residual_right),
        'depth_residual_mm_right_pose_median': round(float(np.median(residual_right)), 1) if residual_right else None,
        'depth_residual_mm_right_pose_min_max': [min(residual_right), max(residual_right)] if residual_right else None,
        'depth_residual_mm_wrong_pose_min_max': [min(residual_wrong), max(residual_wrong)] if residual_wrong else None,
    }
    print(json.dumps(summary, indent=1))
    if args.json:
        Path(args.json).write_text(json.dumps({'summary': summary, 'frames': per_frame}, indent=1))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('colour', help='measure the glove colour near the tags and what else in the scene shares it')
    c.add_argument('run')
    c.add_argument('--min-s', type=int, default=60)
    c.add_argument('--min-v', type=int, default=40)
    c.add_argument('--hue', type=int, nargs=2, metavar=('LO', 'HI'))
    c.add_argument('--range', type=int, nargs=6, metavar=('H', 'S', 'V', 'H', 'S', 'V'), help='candidate lower and upper HSV')
    c.add_argument('--min-area', type=int, default=25)
    c.add_argument('--json')
    s = sub.add_parser('sample', help='HSV of in-range pixels inside named boxes of one saved frame')
    s.add_argument('run')
    s.add_argument('frame', help='e.g. frame_033')
    s.add_argument('--range', type=int, nargs=6, required=True, metavar=('H', 'S', 'V', 'H', 'S', 'V'))
    s.add_argument('--box', nargs=5, action='append', required=True, metavar=('NAME', 'X0', 'Y0', 'X1', 'Y1'))
    s.add_argument('--json')
    e = sub.add_parser('evaluate', help='run the detector with a parameter set on every saved frame')
    e.add_argument('run')
    e.add_argument('--params', required=True)
    e.add_argument('--set', action='append', metavar='KEY=VALUE', help='override a parameter (YAML value)')
    e.add_argument('--images', help='write annotated frames here')
    e.add_argument('--title')
    e.add_argument('--scale', type=float, default=0.5)
    e.add_argument('--world', action='store_true', help='report overhead world positions (default for overhead runs)')
    e.add_argument('--json')
    args = parser.parse_args()
    {'colour': cmd_colour, 'sample': cmd_sample, 'evaluate': cmd_evaluate}[args.cmd](args)


if __name__ == '__main__':
    main()
