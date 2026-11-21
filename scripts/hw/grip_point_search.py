#!/usr/bin/env python3
"""Calibrate each corner's grip point from a recording (`fabric_run.sh grip-search <run_dir> ...`).

The sheet keeps the strip-measured material of the cell config (twin.yaml: bending 1.4e-5 N m from the cantilever
strips, measured areal density; stretch and shear nominal). Each corner is held by one pinned particle (a point
grip), and each corner's grip point is fitted on its own, 4 numbers per corner:

    down_<c>   pinch lower than measured (robot fabric_grip TF, glove estimate), m
    in_<c>     pinch toward the opposite edge (robots toward the hands, hands toward the robots), m
    side_<c>   pinch along the robot edge toward Silver, m
    inset_<c>  material inset of the pinch from the sheet's corner, along both edges, m (mesh-node steps)

for c in blue, silver (robots), right, left (the person's hands). Every candidate free-runs the twin from the
recorded grips over the window and is scored on the raw depth (fabric_twin/identify.py); the objective is the RMS
of the bounded residual 5 cm * tanh(r / 5 cm), points off the sheet counting as 5 cm.

Search: compass search. Each pass tries +step and -step on every parameter (in parallel worker processes), then
cumulative combinations of the best 2, 3, 4, 6, 8 and all improving moves; the best candidate is kept. Offset steps
halve when a pass finds nothing (2 cm down to 5 mm); insets move one mesh node. Every evaluation is logged.
Afterwards the start and the result are scored on held-out windows (--validate run:window ...).

GPU; no ROS graph, no robots. With CUDA MPS running, the workers share the GPU efficiently.
"""
import argparse
import dataclasses
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/fabric_twin'))
sys.path.insert(0, str(REPO / 'scripts/hw'))

CORNERS = ('blue', 'silver', 'right', 'left')
KINDS = ('down', 'in', 'side', 'inset')

_state = {}


def _spec(config, override):
    from fabric_twin.node import load_twin_config
    spec = load_twin_config(Path(config))[0]
    ints = ('cells_x', 'cells_y', 'substeps', 'iterations', 'linear_iterations')
    return dataclasses.replace(spec, **{k: (int(v) if k in ints else float(v)) for k, v in override.items()})


def _init(config, override, names, run, window, depth_every, points):
    import identify_replay as ir
    from fabric_twin.identify import Forward
    spec = _spec(config, override)
    _state['forward'] = Forward(spec, ir.load(Path(run), window, depth_every, points, 1.), names, point_grips=True)


def _evaluate(theta):
    from fabric_twin.identify import summary
    return summary(_state['forward'].raw(np.asarray(theta, float)[None]))[0]


def objective(result):
    return result['rms_tanh50_mm']


class Pool:
    """Evaluations of one window, in `workers` processes that keep their models between calls."""
    def __init__(self, workers, *init):
        self.pool = mp.get_context('spawn').Pool(workers, initializer=_init, initargs=init)

    def map(self, thetas):
        return self.pool.map(_evaluate, [list(t) for t in thetas], chunksize=1)

    def close(self):
        self.pool.close(); self.pool.join()


def safe_workers(requested, per_worker_gb=6., reserve_gb=4.):
    """Worker processes that fit in the memory available now. Each GPU worker (Warp, Newton, CUDA context, one model)
    peaked at 5.7 GB of host memory on 2026-11-21; six of them plus the rest ran the 30 GB laptop out of memory."""
    try:
        available = next(int(line.split()[1]) for line in open('/proc/meminfo') if line.startswith('MemAvailable'))/1e6
    except (OSError, StopIteration):
        return requested
    fit = max(1, int((available-reserve_gb)//per_worker_gb))
    if fit < requested:
        print(f'{requested} workers requested; {available:.1f} GB available allows {fit} '
              f'({per_worker_gb:.0f} GB each, {reserve_gb:.0f} GB kept free)', flush=True)
    return min(requested, fit)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', type=Path)
    ap.add_argument('--window', default='25:49')
    ap.add_argument('--config', default=str(REPO / 'experiments/hardware/cell/twin.yaml'))
    ap.add_argument('--override', nargs='*', default=['cells_x=40', 'cells_y=40', 'substeps=32', 'iterations=10'])
    ap.add_argument('--start', nargs='*', default=['down_right=0.043', 'down_left=0.043'],
                    help='starting values, e.g. the earlier shared hand fit')
    ap.add_argument('--corners', default=','.join(CORNERS))
    ap.add_argument('--kinds', default=','.join(KINDS))
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--step', type=float, default=.02)
    ap.add_argument('--min-step', type=float, default=.005)
    ap.add_argument('--max-passes', type=int, default=12)
    ap.add_argument('--tolerance-mm', type=float, default=.1, help='smallest objective gain that counts')
    ap.add_argument('--validate', nargs='*', default=[], help='held-out run_dir:window pairs')
    ap.add_argument('--depth-every', type=int, default=2)
    ap.add_argument('--points', type=int, default=64)
    ap.add_argument('--out', type=Path, required=True, help='output folder (log.jsonl, result.json)')
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    override = dict(item.split('=') for item in a.override)
    names = [f'{k}_{c}' for c in a.corners.split(',') for k in a.kinds.split(',')]
    from fabric_twin.identify import PARAMETERS
    spec = _spec(a.config, override)
    node = spec.width_m/spec.cells_x
    theta = np.zeros(len(names))
    for item in a.start:
        key, value = item.split('=')
        if key in names:
            theta[names.index(key)] = float(value)
    lower = np.array([PARAMETERS[n].lower for n in names]); upper = np.array([PARAMETERS[n].upper for n in names])
    is_inset = np.array([n.startswith('inset_') for n in names])
    step = np.where(is_inset, node, a.step)
    log = open(a.out / 'log.jsonl', 'w')
    a.workers = safe_workers(a.workers)
    pool = Pool(a.workers, a.config, override, names, str(a.run.resolve()), a.window, a.depth_every, a.points)
    begun = time.perf_counter()

    def record(kind, t, r):
        row = dict(kind=kind, seconds=round(time.perf_counter()-begun, 1), values=dict(zip(names, np.round(t, 4).tolist())), **r)
        log.write(json.dumps(row)+'\n'); log.flush()
        return row

    best = pool.map([theta])[0]
    start_result = record('start', theta, best)
    print(f'start: objective {objective(best):.2f} mm, median {best["median_mm"]:.1f} mm, p95 {best["p95_abs_mm"]:.1f} mm '
          f'({len(names)} parameters, node {node*100:.1f} cm)', flush=True)
    for npass in range(a.max_passes):
        trials, where = [], []
        for i in range(len(names)):
            for sign in (1., -1.):
                t = theta.copy(); t[i] = np.clip(t[i]+sign*step[i], lower[i], upper[i])
                if t[i] != theta[i]:
                    trials.append(t); where.append((i, sign))
        results = pool.map(trials)
        for t, r in zip(trials, results):
            record(f'pass{npass}', t, r)
        gains = {}
        for (i, sign), r in zip(where, results):
            gain = objective(best)-objective(r)
            if gain > a.tolerance_mm and gain > gains.get(i, (0, 0, None))[0]:
                gains[i] = (gain, sign, r)
        if not gains:
            if np.all(step[~is_inset] <= a.min_step+1e-9):
                print(f'pass {npass}: no improving move at the smallest step; done', flush=True)
                break
            step[~is_inset] = np.maximum(step[~is_inset]/2, a.min_step)
            print(f'pass {npass}: no improving move; offset step -> {step[~is_inset][0]*100:.2f} cm', flush=True)
            continue
        # Candidates: the best single move, and cumulative combinations of the best 2, 3, 4, 6, 8 and all improving
        # moves (applying all ten at once overshot on hardware data), evaluated together in one parallel batch.
        ranked = sorted(gains.items(), key=lambda kv: -kv[1][0])
        i0, (_, s0, r0) = ranked[0]
        single = theta.copy(); single[i0] = np.clip(theta[i0]+s0*step[i0], lower[i0], upper[i0])
        candidates = [(single, r0)]
        combos = []
        for k in sorted({2, 3, 4, 6, 8, len(ranked)}):
            if k <= len(ranked) and k > 1:
                c = theta.copy()
                for i, (_, sign, _) in ranked[:k]:
                    c[i] = np.clip(c[i]+sign*step[i], lower[i], upper[i])
                combos.append(c)
        if combos:
            for c, r in zip(combos, pool.map(combos)):
                record(f'pass{npass}-combined', c, r)
                candidates.append((c, r))
        theta, best = min(candidates, key=lambda c: objective(c[1]))
        print(f'pass {npass}: objective {objective(best):.2f} mm, median {best["median_mm"]:.1f}, p95 {best["p95_abs_mm"]:.1f}; '
              f'{len(gains)} improving coordinates; ' + ', '.join(f'{n}={v*100:.1f}cm' for n, v in zip(names, theta) if v), flush=True)
    pool.close()
    final = record('final', theta, best)
    validation = []
    start = np.zeros(len(names))
    for item in a.start:
        key, value = item.split('=')
        if key in names:
            start[names.index(key)] = float(value)
    for item in a.validate:
        run, window = item.rsplit(':', 2)[0], ':'.join(item.rsplit(':', 2)[1:])
        vpool = Pool(min(a.workers, 3), a.config, override, names, str(Path(run).resolve()), window, a.depth_every, a.points)
        corner, began, fitted = vpool.map([np.zeros(len(names)), start, theta])
        vpool.close()
        validation.append(dict(run=run, window=window, point_grips_at_corners=corner, start=began, calibrated=fitted))
        print(f'held out {Path(run).name} {window}: corners {objective(corner):.1f} / start {objective(began):.1f} / '
              f'calibrated {objective(fitted):.1f} mm (median {corner["median_mm"]:.0f} / {began["median_mm"]:.0f} / '
              f'{fitted["median_mm"]:.0f}, p95 {corner["p95_abs_mm"]:.0f} / {began["p95_abs_mm"]:.0f} / {fitted["p95_abs_mm"]:.0f})', flush=True)
    result = dict(run=str(a.run), window=a.window, override=override, node_m=node, names=names,
                  calibrated={n: float(v) for n, v in zip(names, theta)}, start=start_result, final=final,
                  validation=validation, evaluations=sum(1 for _ in open(a.out / 'log.jsonl')),
                  seconds=time.perf_counter()-begun, material='twin.yaml as measured (bend 1.4e-5 N m strips, density 0.215)')
    (a.out / 'result.json').write_text(json.dumps(result, indent=1))
    print(json.dumps(result['calibrated'], indent=1))


if __name__ == '__main__':
    main()
