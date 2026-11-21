#!/usr/bin/env python3
"""Fast two-stage fit on a recording's still hold (`fabric_run.sh static-fit <run> [<run> ...]`), 2026-11-21.

For each run on its own: (1) the four grip points (down/in/side, 12 numbers), corner-weighted; (2) the stretch scale
with those grips held. Levenberg-Marquardt with a parallel finite-difference Jacobian (fabric_twin/static_fit.py).
Grip points are per hold (2026-11-21: run 1's fitted grips made run 2 worse, 34.1 -> 37.0 mm), so they are refitted
every time; stretch is a property of the cloth, so it should come out the same from every run: that is the check.
Same model as grip-fit: point grips, 40 x 40 at 32 x 10 by default. Writes <run>/static_fit/result.json.
"""
import argparse
import dataclasses
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
GRIPS = [f'{k}_{c}' for c in ('blue', 'silver', 'right', 'left') for k in ('down', 'in', 'side')]
_state = {}


def _spec(config, override):
    from fabric_twin.node import load_twin_config
    spec = load_twin_config(Path(config))[0]
    ints = ('cells_x', 'cells_y', 'substeps', 'iterations', 'linear_iterations')
    return dataclasses.replace(spec, **{k: (int(v) if k in ints else float(v)) for k, v in override.items()})


def _init(config, override, run, window, point_grips=True):
    import identify_replay as ir
    _state.update(spec=_spec(config, override), window=ir.load(Path(run), window, 1, 128, 1.), forwards={},
                  point=point_grips)


def _raw(task):
    """task = (names, fixed, theta): raw residuals of one hypothesis (one model per parameter set, kept)."""
    from fabric_twin.identify import Forward
    names, fixed, theta = task
    key = (tuple(names), tuple(sorted(fixed.items())))
    if key not in _state['forwards']:
        f = Forward(_state['spec'], _state['window'], names, point_grips=_state['point'])
        f.set_fixed(**fixed)
        _state['forwards'] = {key: f}          # one model per worker (memory)
    return _state['forwards'][key].raw(np.asarray(theta, float)[None])[0]


def fit_run(run, a, override, workers):
    import identify_replay as ir
    from fabric_twin.identify import PARAMETERS
    from fabric_twin.static_fit import corner_weights, levenberg_marquardt
    window = ir.load(Path(run), a.window, 1, 128, 1.)
    pool = mp.get_context('spawn').Pool(workers, initializer=_init,
                                        initargs=(a.config, override, str(run), a.window, not a.patch_grips))
    out = {}
    try:
        def stage(names, fixed, weights, theta0, steps, label):
            print(f'  {label}: {len(names)} parameters', flush=True)
            res = levenberg_marquardt(
                lambda thetas: pool.map(_raw, [(names, fixed, t) for t in thetas], chunksize=1),
                theta0, names, [PARAMETERS[n].lower for n in names], [PARAMETERS[n].upper for n in names], steps,
                weights=weights, max_iterations=a.max_iterations,
                progress=lambda h: print(f'    iteration {h["iteration"]}: {h["rms_mm"]:.2f} mm (damping {h["damping"]:g}, '
                                         f'{h["seconds"]:.0f} s)', flush=True))
            print(f'  {label}: {res.start_cost_mm:.2f} -> {res.cost_mm:.2f} mm, {res.simulations} simulations, '
                  f'{res.seconds:.0f} s, {res.stop_reason}', flush=True)
            return res
        previous = Path(run) / 'static_fit' / 'result.json'
        if a.reuse_grips and previous.exists():
            from fabric_twin.static_fit import FitResult
            d = json.loads(previous.read_text())['grips']
            g = FitResult(d['names'], np.array([d['values'][n] for n in d['names']]), np.array([d['sd'][n] for n in d['names']]),
                          d['cost_mm'], d['start_cost_mm'], d['iterations'], 0, 0., d['converged'], 'reused', d['history'])
            print(f'  stage 1 grips reused from {previous}', flush=True)
        else:
            g = stage(GRIPS, {}, corner_weights(window, a.near_m, a.far_weight), np.zeros(len(GRIPS)),
                      np.full(len(GRIPS), a.grip_step), 'stage 1 grips (corner-weighted)')
        for n, v, s in zip(GRIPS, g.theta, g.sd):
            if abs(v) > 1e-4:
                print(f'    {n} {v*100:+.1f} cm (sd {s*100:.1f})')
        from fabric_twin.static_fit import line_search_1d
        print('  stage 2 stretch (grips held): parallel line search', flush=True)
        fixed = g.values()
        m = line_search_1d(lambda thetas: pool.map(_raw, [(['stretch'], fixed, t) for t in thetas], chunksize=1),
                           a.stretch_range[0], a.stretch_range[1], points=a.workers,
                           progress=lambda h: print(f'    sweep {h["iteration"]}: best x{10**h["value"]:.3f}, '
                                                    f'{h["rms_mm"]:.2f} mm ({h["seconds"]:.0f} s)', flush=True))
        print(f'  stage 2 stretch: {m.start_cost_mm:.2f} -> {m.cost_mm:.2f} mm, {m.simulations} simulations, {m.seconds:.0f} s')
        print(f'    stretch x{10**m.theta[0]:.3f} (log10 {m.theta[0]:+.3f})')
        out = dict(run=str(run), window=a.window, override=override, grips=g.as_dict(), stretch=m.as_dict(),
                   stretch_scale=float(10**m.theta[0]), seconds=round(g.seconds+m.seconds, 1))
    finally:
        pool.close(); pool.join()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', type=Path)
    ap.add_argument('--window', default='still:3')
    ap.add_argument('--config', default=str(REPO / 'experiments/hardware/cell/twin.yaml'))
    ap.add_argument('--override', nargs='*', default=['cells_x=40', 'cells_y=40', 'substeps=32', 'iterations=10'])
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--near-m', type=float, default=.2)
    ap.add_argument('--far-weight', type=float, default=.1)
    ap.add_argument('--grip-step', type=float, default=.005, help='finite-difference step, m')
    ap.add_argument('--stretch-range', type=float, nargs=2, default=[-2.5, .5], help='log10 search range of the stretch scale')
    ap.add_argument('--max-iterations', type=int, default=8)
    ap.add_argument('--reuse-grips', action='store_true', help="take stage 1 from the run's static_fit/result.json")
    ap.add_argument('--patch-grips', action='store_true',
                    help="square patch grips (nodes within patch_m of the corner; e.g. --override patch_m=0.06 = 3 x 3 nodes "
                         "on 28 x 28) instead of one particle per corner")
    ap.add_argument('--tag', default='static_fit', help='output folder name inside each run (default static_fit)')
    a = ap.parse_args()
    override = dict(item.split('=') for item in a.override)
    results = []
    for run in a.runs:
        print(f'== {run.name}', flush=True)
        begun = time.perf_counter()
        r = fit_run(run.resolve(), a, override, a.workers)
        r['wall_seconds'] = round(time.perf_counter()-begun, 1)
        (run / a.tag).mkdir(exist_ok=True)
        (run / a.tag / 'result.json').write_text(json.dumps(r, indent=1))
        print(f'  total {r["wall_seconds"]:.0f} s; written {run / a.tag / "result.json"}', flush=True)
        results.append(r)
    if len(results) > 1:
        s = [r['stretch']['values']['stretch'] for r in results]
        sd = [r['stretch']['sd']['stretch'] for r in results]
        print(f'stretch across runs (log10): {np.round(s, 3).tolist()} (sd {np.round(sd, 3).tolist()}); '
              f'scales {np.round([10**x for x in s], 3).tolist()}; spread {max(s)-min(s):.3f} decades')


if __name__ == '__main__':
    main()
