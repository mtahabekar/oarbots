#!/usr/bin/env python3
"""Score a recorded run against the cameras, independently of the twin (`fabric_run.sh audit <run_dir>`).

The twin's clearance is a prediction; the paper's safety number must come from the real sheet. This reads
the run's twin.npz, twin.depth.npz, estimate.npz and twin.cbf.jsonl(.gz) (run folder or its
/home/taha/fabric_hw_data copy) and reports:

  1. twin vs depth: signed distance of every recorded sheet depth point to the twin surface at that time
     (negative = real sheet below the twin), per camera, and a map over the sheet's material coordinates
  2. estimator vs depth: the same for the estimator's mesh (upsampled bilinearly)
  3. with a detected obstacle (cell/obstacle.yaml): the real sheet's clearance to the CBF capsule from depth
     (third-closest point per frame), the estimator's, and the twin's (CBF log), while commands were on;
     and how many frames had sheet depth points inside the obstacle's prism (likely contact; check images)

Writes audit.json into the run folder. No ROS graph, no GPU.
"""
import argparse
import gzip
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import zoom
from scipy.spatial import cKDTree

GATE_M = .15


def data_file(run, name, required=True):
    for folder in (run, Path('/home/taha/fabric_hw_data') / f'{run.parent.name}_{run.name}'):
        if (folder / name).exists():
            return folder / name
    if required:
        raise SystemExit(f'{name} not found for {run} (nor in /home/taha/fabric_hw_data)')
    return None


def surface(grid_q):
    """Flattened points and upward unit normals of a (rows, cols, 3) surface grid."""
    normal = np.cross(np.gradient(grid_q, axis=1), np.gradient(grid_q, axis=0))
    normal /= np.maximum(np.linalg.norm(normal, axis=2, keepdims=True), 1e-12)
    flat, normals = grid_q.reshape(-1, 3), normal.reshape(-1, 3)
    if normals[:, 2].mean() < 0:
        normals = -normals
    return flat, normals


def signed(flat, normals, points):
    distance, index = cKDTree(flat).query(points)
    keep = distance < GATE_M
    return np.einsum('ij,ij->i', points[keep] - flat[index[keep]], normals[index[keep]]), index[keep], keep


def stats(r):
    if not len(r):
        return None
    return dict(points=int(len(r)), median_mm=round(float(np.median(r)) * 1e3, 1),
                mad_mm=round(float(np.median(np.abs(r - np.median(r)))) * 1e3, 1),
                p95_abs_mm=round(float(np.percentile(np.abs(r), 95)) * 1e3, 1),
                over_30mm=round(float(np.mean(np.abs(r) > .03)), 3))


def capsule_distance(points, capsule):
    cx, cy = capsule['center'][:2]
    radius, height = capsule['radius'], capsule['cylinder_height'] + capsule.get('cap_height', 0.)
    rho = np.hypot(points[:, 0] - cx, points[:, 1] - cy)
    dz = points[:, 2] - height
    return np.where(dz > 0, np.where(rho <= radius, dz, np.hypot(rho - radius, dz)), rho - radius)


def inside_polygon(xy, polygon):
    x, y = xy[:, 0], xy[:, 1]
    inside = np.zeros(len(xy), bool)
    for (x1, y1), (x2, y2) in zip(polygon, np.roll(polygon, -1, axis=0)):
        crosses = (y1 > y) != (y2 > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            at = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        inside ^= crosses & (x < at)
    return inside


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', type=Path)
    ap.add_argument('--bins', type=int, default=6, help='material-coordinate map size')
    a = ap.parse_args()
    run = a.run.resolve()
    twin = np.load(data_file(run, 'twin.npz'))
    depth = np.load(data_file(run, 'twin.depth.npz'))
    estimate_path = data_file(run, 'estimate.npz', required=False)
    offsets = np.r_[0, np.cumsum(depth['counts'])]
    uv = twin['uv'].astype(float)
    n = int(round(np.sqrt(len(uv))))
    grid = np.lexsort((uv[:, 0], uv[:, 1])).reshape(n, n)
    result = dict(run=str(run))

    # 1. twin vs depth, per camera and as a material map
    per_camera, bins = {}, a.bins
    cells = [[[] for _ in range(bins)] for _ in range(bins)]
    for j in range(len(depth['stamps_ns'])):
        k = int(np.argmin(abs(twin['times_ns'] - depth['stamps_ns'][j])))
        if abs(twin['times_ns'][k] - depth['stamps_ns'][j]) > .05e9:
            continue
        flat, normals = surface(twin['positions'][k].astype(float)[grid])
        r, index, _ = signed(flat, normals, depth['points'][offsets[j]:offsets[j + 1]].astype(float))
        per_camera.setdefault(int(depth['camera'][j]), []).append(r)
        where = uv[grid.reshape(-1)[index]]
        bu = np.minimum((where[:, 0] * bins).astype(int), bins - 1)
        bv = np.minimum((where[:, 1] * bins).astype(int), bins - 1)
        for row, col, value in zip(bv, bu, r):
            cells[row][col].append(value)
    result['twin_vs_depth'] = {str(c): stats(np.concatenate(v)) for c, v in per_camera.items()}
    result['twin_vs_depth_map_mm'] = [[round(float(np.median(c)) * 1e3) if len(c) > 50 else None for c in row] for row in cells]

    # 2. estimator vs depth
    estimate = None
    if estimate_path:
        estimate = np.load(estimate_path)
        use = ~estimate['predicted']
        e_ns, e_q = estimate['measurement_ns'][use], estimate['positions'][use]
        m = int(round(np.sqrt(e_q.shape[1])))
        per_camera = {}
        for j in range(len(depth['stamps_ns'])):
            k = int(np.argmin(abs(e_ns - depth['stamps_ns'][j])))
            if abs(e_ns[k] - depth['stamps_ns'][j]) > .05e9:
                continue
            q = e_q[k].reshape(m, m, 3)
            fine = np.stack([zoom(q[..., c], (n - 1) / (m - 1) + 1e-9, order=1) for c in range(3)], -1)
            flat, normals = surface(fine)
            r, _, _ = signed(flat, normals, depth['points'][offsets[j]:offsets[j + 1]].astype(float))
            per_camera.setdefault(int(depth['camera'][j]), []).append(r)
        result['estimate_vs_depth'] = {str(c): stats(np.concatenate(v)) for c, v in per_camera.items()}
        strain = np.linalg.norm(np.diff(e_q.reshape(-1, m, m, 3), axis=2), axis=-1) / (.84 / (m - 1)) - 1
        result['estimate_edge_strain_p95'] = round(float(np.percentile(strain, 95)), 3)

    # 3. clearance to the detected obstacle
    obstacle_file = run / 'cell' / 'obstacle.yaml'
    obstacle = yaml.safe_load(obstacle_file.read_text()).get('obstacle', {}) if obstacle_file.exists() else {}
    capsule = obstacle.get('capsule')
    if capsule:
        cbf_path = run / 'twin.cbf.jsonl.gz' if (run / 'twin.cbf.jsonl.gz').exists() else run / 'twin.cbf.jsonl'
        rows = []
        with (gzip.open(cbf_path, 'rt') if cbf_path.suffix == '.gz' else open(cbf_path)) as f:
            for line in f:
                d = json.loads(line)
                if d.get('twin_time') is not None and d.get('min_clearance') is not None:
                    rows.append((d['twin_time'], d['min_clearance'], float(bool(d.get('commands_enabled')))))
        cbf = np.array(rows, float)
        prism = obstacle.get('prism') or {}
        footprint = np.array(prism.get('footprint', []), float)
        real, inside_frames = [], 0
        for j in range(len(depth['stamps_ns'])):
            p = depth['points'][offsets[j]:offsets[j + 1]].astype(float)
            if len(p) < 50:
                continue
            t = depth['stamps_ns'][j] / 1e9
            k = int(np.argmin(abs(cbf[:, 0] - t)))
            if abs(cbf[k, 0] - t) > .1:
                continue
            real.append((np.sort(capsule_distance(p, capsule))[2], cbf[k, 1], cbf[k, 2]))
            if len(footprint) >= 3 and cbf[k, 2] > 0:
                inside = inside_polygon(p[:, :2], footprint) & (p[:, 2] < prism['top'])
                inside_frames += int(inside.sum() >= 3)
        real = np.array(real)
        if len(real):   # every frame, commands on or off (a shadow run has them off)
            gap = real[:, 0] - real[:, 1]
            result['clearance_all_frames'] = dict(
                frames=int(len(real)), twin_min_m=round(float(real[:, 1].min()), 3),
                real_p5_m=round(float(np.percentile(real[:, 0], 5)), 3), real_min_m=round(float(real[:, 0].min()), 3),
                real_minus_twin_median_m=round(float(np.median(gap)), 3), real_minus_twin_p5_m=round(float(np.percentile(gap, 5)), 3))
        on = real[:, 2] > 0 if len(real) else np.zeros(0, bool)
        if on.any():
            gap = real[on, 0] - real[on, 1]
            result['clearance_commands_on'] = dict(
                frames=int(on.sum()), twin_min_m=round(float(real[on, 1].min()), 3),
                real_p5_m=round(float(np.percentile(real[on, 0], 5)), 3), real_min_m=round(float(real[on, 0].min()), 3),
                real_minus_twin_median_m=round(float(np.median(gap)), 3), real_minus_twin_p5_m=round(float(np.percentile(gap, 5)), 3),
                frames_real_below_zero=int((real[on, 0] < 0).sum()), frames_with_depth_inside_prism=inside_frames,
                note='real = third-closest sheet depth point to the CBF capsule per frame; check images before calling contact')
        if estimate is not None:
            ce = []
            for t, q in zip(e_ns / 1e9, e_q):
                k = int(np.argmin(abs(cbf[:, 0] - t)))
                if abs(cbf[k, 0] - t) < .1 and cbf[k, 2] > 0:
                    fine = np.stack([zoom(q.reshape(m, m, 3)[..., c], (n - 1) / (m - 1) + 1e-9, order=1) for c in range(3)], -1)
                    ce.append((capsule_distance(fine.reshape(-1, 3), capsule).min(), cbf[k, 1]))
            if ce:
                ce = np.array(ce)
                result['estimate_clearance_commands_on'] = dict(p5_m=round(float(np.percentile(ce[:, 0], 5)), 3),
                                                                minus_twin_median_m=round(float(np.median(ce[:, 0] - ce[:, 1])), 3))
    print(json.dumps(result, indent=1))
    (run / 'audit.json').write_text(json.dumps(result, indent=1))
    print(f'written {run / "audit.json"}', file=sys.stderr)


if __name__ == '__main__':
    main()
