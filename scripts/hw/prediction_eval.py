#!/usr/bin/env python3
"""E2: how well the twin predicts the sheet (`fabric_run.sh prediction-eval <run> [<run> ...] [options]`), 2026-11-21.

For start times through a run (every --every s): take the live twin's recorded state (twin.npz), rebuild the twin with
the run's own configuration and obstacle (cell/twin.yaml, cell/obstacle.yaml), switch off the depth correction, and
free-run it on the recorded grips (60 Hz) for each horizon. Score against the raw depth captured at that moment
(twin.depth.npz, within 40 ms), per camera. Two baselines on the same points:
  hold  the state at the start compared with the later depth (no dynamics);
  live  the live twin's own recorded state at the later time (corrected by the estimator, what the CBF saw).
`--config` swaps the sheet model (same mesh required, e.g. a TWIN=nominal copy) to compare models on one recording.
Reports per horizon and method: median signed residual (point minus sheet, + = point above the sheet), MAD, p95 |r|,
RMS of the bounded residual 5 cm * tanh(r / 5 cm). Writes <run>/prediction_eval[_<tag>].json. GPU; no ROS graph.
"""
import argparse
import dataclasses
import json
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from identify_replay import data_file  # noqa: E402


def stats(r):
    r = r[np.isfinite(r)]
    if len(r) < 20:
        return None
    return dict(points=int(len(r)), median_mm=round(float(np.median(r))*1e3, 1),
                mad_mm=round(float(np.median(np.abs(r-np.median(r))))*1e3, 1),
                p95_abs_mm=round(float(np.percentile(np.abs(r), 95))*1e3, 1),
                rms_tanh50_mm=round(float(np.sqrt(np.mean((.05*np.tanh(r/.05))**2)))*1e3, 1))


def obstacle_for(run, spec):
    from fabric_twin.cloth import Obstacle
    path = run / 'cell' / 'obstacle.yaml'
    if not path.exists():
        return spec
    detected = yaml.safe_load(path.read_text())['obstacle']
    if detected.get('shape') == 'prism_capsule':   # as fabric_twin.node does
        prism, capsule = detected['prism'], detected['capsule']
        foot = np.asarray(prism['footprint'], float)
        lo, hi = np.r_[foot.min(0), 0.], np.r_[foot.max(0), prism['top']]
        box = Obstacle(center=tuple((lo+hi)/2), half_extents=tuple((hi-lo)/2), yaw=0., prism=prism, capsule=capsule)
    else:
        box = Obstacle(**{k: tuple(v) if isinstance(v, list) else v for k, v in detected.items()})
    return dataclasses.replace(spec, obstacle=box)


def evaluate(run, a):
    from fabric_twin.cloth import TwinCloth
    from fabric_twin.identify import signed_residuals
    from fabric_twin.node import load_twin_config
    config = Path(a.config) if a.config else run / 'cell' / 'twin.yaml'
    spec = obstacle_for(run, load_twin_config(config)[0])
    twin = np.load(data_file(run, 'twin.npz'))
    depth = np.load(data_file(run, 'twin.depth.npz'))
    t_ns, pos, vel = twin['times_ns'], twin['positions'].astype(float), twin['velocities'].astype(float)
    c_ns, corners = twin['corners_ns'], twin['corners'].astype(float)
    d_ns, cams, counts = depth['stamps_ns'], depth['camera'], depth['counts']
    all_points = depth['points']
    offsets = np.r_[0, np.cumsum(counts)]
    cloth = TwinCloth(spec, corners[0])
    if cloth.count != pos.shape[1]:
        sys.exit(f'{config}: {cloth.count} particles, but the recording has {pos.shape[1]}: the mesh must match the run')
    nx, ny = spec.cells_x+1, spec.cells_y+1
    grid = np.lexsort((cloth.uv[:, 0], cloth.uv[:, 1])).reshape(ny, nx)
    horizons = sorted(a.horizons)
    out = {h: {m: {} for m in ('model', 'hold', 'live')} for h in horizons}
    residuals = {h: {m: {} for m in ('model', 'hold', 'live')} for h in horizons}
    starts = np.arange(t_ns[0]+2e9, t_ns[-1]-horizons[-1]*1e9-.2e9, a.every*1e9)
    used = 0
    for t0 in starts:
        i = int(np.searchsorted(t_ns, t0, side='right'))-1
        if i < 0:
            continue
        t0 = int(t_ns[i])
        frames = np.flatnonzero((c_ns > t0) & (c_ns <= t0+horizons[-1]*1e9+.02e9))
        if len(frames) < horizons[-1]*50:          # grips missing in this stretch
            continue
        cloth.set_external(np.zeros((cloth.count, 3)))
        cloth.targets = cloth.to_twin(corners[frames[0]])
        cloth.set_state(pos[i], vel[i])
        predicted, k = {}, 0
        for j in frames:
            cloth.step(corners[j])
            while k < len(horizons) and c_ns[j] >= t0+horizons[k]*1e9:
                predicted[horizons[k]] = cloth.state()[0].copy()
                k += 1
        if len(predicted) < len(horizons):
            continue
        used += 1
        for h in horizons:
            th = t0+h*1e9
            near = np.flatnonzero(np.abs(d_ns-th) <= .04e9)
            j_live = int(np.argmin(np.abs(t_ns-th)))
            for d in near:
                p = all_points[offsets[d]:offsets[d+1]].astype(float)
                if len(p) < 20:
                    continue
                if len(p) > a.points:
                    p = p[np.random.default_rng(d).choice(len(p), a.points, replace=False)]
                cam = int(cams[d])
                for method, q in (('model', predicted[h]), ('hold', pos[i]), ('live', pos[j_live])):
                    residuals[h][method].setdefault(cam, []).append(signed_residuals(q, grid, p))
    for h in horizons:
        for method in out[h]:
            for cam, rs in residuals[h][method].items():
                out[h][method][str(cam)] = stats(np.concatenate(rs))
            if residuals[h][method]:
                out[h][method]['all'] = stats(np.concatenate([np.concatenate(rs) for rs in residuals[h][method].values()]))
    return dict(run=str(run), config=str(config), mesh=[spec.cells_x, spec.cells_y], solver=[spec.substeps, spec.iterations],
                stretch=list(spec.stretch), starts=used, every_s=a.every, horizons_s=horizons,
                results={str(h): out[h] for h in horizons})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', type=Path)
    ap.add_argument('--horizons', type=float, nargs='+', default=[.1, .33, 1.])
    ap.add_argument('--every', type=float, default=1., help='seconds between start times')
    ap.add_argument('--points', type=int, default=256, help='depth points per frame (sampled)')
    ap.add_argument('--config', help="a twin.yaml for the sheet model (default: the run's own)")
    ap.add_argument('--tag', default='', help='output name suffix, e.g. nominal')
    a = ap.parse_args()
    for run in a.runs:
        run = run.resolve()
        r = evaluate(run, a)
        name = 'prediction_eval' + (f'_{a.tag}' if a.tag else '') + '.json'
        (run / name).write_text(json.dumps(r, indent=1))
        print(f'== {run.name}: {r["starts"]} starts, mesh {r["mesh"]}, solver {r["solver"]}, stretch {r["stretch"][0]:g}')
        print(f'{"horizon":>8} {"method":>6}  median   MAD   p95|r|  rms-tanh (mm, all cameras)')
        for h, by in r['results'].items():
            for method, s in by.items():
                s = s.get('all') if s else None
                if s:
                    print(f'{h:>8} {method:>6}  {s["median_mm"]:6.1f} {s["mad_mm"]:5.1f} {s["p95_abs_mm"]:7.1f} {s["rms_tanh50_mm"]:8.1f}')
        print(f'written {run / name}')


if __name__ == '__main__':
    main()
