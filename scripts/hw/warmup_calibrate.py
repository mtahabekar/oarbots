#!/usr/bin/env python3
"""Calibrate the twin from a warm-up recording and test, factor by factor, what actually reduces the error
(`fabric_run.sh warmup-calibrate <warmup_run> --out <dir> ...`).

What the warm-up detects (each from the warm-up window alone, all other factors at their baseline):

  R  resolution  the coarsest mesh whose warm-up error is within --resolution-tol-mm of the finest candidate
  G  grip points each corner's pinch point (offset from robot TF / glove estimate, inset from the sheet corner),
                 from scripts/hw/grip_point_search.py (pass its result.json with --grips)
  B  bending     log10 scale of the strip-measured bending stiffness, by a 1-D scan
  S  shear       log10 scale of the nominal shear stiffness, by a 1-D scan

A factor counts as detectable when its scan changes the warm-up objective by more than --detect-tol-mm and the
best value is not at the edge of the scan.

Test design (each candidate free-runs the twin from the recorded grips and is scored on raw depth; objective =
RMS of 5 cm * tanh(r / 5 cm), off-sheet points counting as 5 cm; solver 32 substeps x 10 iterations; point grips):

  * baseline: finest mesh, corners at their measurements, strip-measured bending, nominal shear
  * all 2^4 combinations of R, G, B, S at their detected values (one factor at a time is the subset with one
    factor on; the full combination has all four)
  * joint: grip points, then bending and shear re-scanned with the grip points in place
  * reference: the twin as deployed on 2026-11-20 (28 x 28, 8 x 6 iterations, 3 cm square patches)

on the warm-up window and on held-out windows (--validate run:window ...). For every factor the main effect is
the mean, over the other factors' 8 combinations, of (with - without); its 95 % interval comes from a moving-block
bootstrap over depth frames (--block-sec blocks), per window. A factor is adopted only if its effect on the
held-out windows is negative (better) with an interval excluding zero on at least one held-out window and
positive-free on all of them.

Writes result.json (everything), table.csv (every configuration x window) and report.md. GPU; no ROS graph.
"""
import argparse
import csv
import dataclasses
import itertools
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
GRIP_NAMES = [f'{k}_{c}' for c in CORNERS for k in ('down', 'in', 'side', 'inset')]
NAMES = GRIP_NAMES + ['bend', 'shear']
SCALE = .05
CONVERGED = dict(substeps=32, iterations=10)
DEPLOYED = dict(cells_x=28, cells_y=28, substeps=8, iterations=6, patch_m=.03)

_cache = {}


def _forward(config, cells, run, window, deployed, numerics=None):
    """The worker's model for one (mesh, window). Only one is kept: with six workers each holding a model per mesh
    and window (and its CUDA graph), the factorial ran the 30 GB laptop out of memory on 2026-11-20 23:38."""
    key = (cells, run, window, deployed, tuple(sorted((numerics or {}).items())))
    if key not in _cache:
        import gc
        _cache.clear(); gc.collect()
        import identify_replay as ir
        from fabric_twin.identify import Forward
        from fabric_twin.node import load_twin_config
        spec = load_twin_config(Path(config))[0]
        spec = dataclasses.replace(spec, **(DEPLOYED if deployed else dict(cells_x=cells, cells_y=cells, **{**CONVERGED, **(numerics or {})})))
        w = ir.load(Path(run), window, 2, 64, 1.)
        _cache[key] = (Forward(spec, w, NAMES, point_grips=not deployed), np.array([len(p) for p in w.points]),
                       np.array([w.frame_ns[i] for i in w.depth_index]))
    return _cache[key]


def _task(task):
    """task: dict(config, cells, run, window, deployed, theta) -> summary + per-frame mean square (mm^2)."""
    import resource
    from fabric_twin.identify import summary
    forward, counts, stamps = _forward(task['config'], task['cells'], task['run'], task['window'], task['deployed'],
                                       task.get('numerics'))
    theta = np.array([task['theta'].get(n, 0.) for n in NAMES])
    raw = forward.raw(theta[None])[0]
    bounded = np.where(np.isfinite(raw), SCALE*np.tanh(np.nan_to_num(raw)/SCALE), SCALE)
    frames = [float(np.mean(x**2)*1e6) for x in np.split(bounded, np.cumsum(counts)[:-1])]
    return dict(summary(raw[None])[0], frame_ms_mm2=frames, frame_t=((stamps-stamps[0])/1e9).tolist(),
                worker_max_rss_gb=round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1e6, 2))


def _group(tasks):
    """All tasks of one (mesh, window), in one worker, so its model is built once."""
    return [_task(t) for t in tasks]


def task_key(t):
    return json.dumps([t['cells'], Path(t['run']).name, t['window'], bool(t.get('deployed', False)),
                       sorted((k, round(float(v), 6)) for k, v in t['theta'].items() if v)])


def objective(r):
    return r['rms_tanh50_mm']


def block_bootstrap(diff, t, block_sec, rounds=2000, seed=0):
    """95 % interval of mean(diff) by a moving-block bootstrap over time-ordered frames."""
    diff, t = np.asarray(diff, float), np.asarray(t, float)
    blocks = np.floor((t-t[0])/block_sec).astype(int)
    ids = np.unique(blocks)
    groups = [diff[blocks == b] for b in ids]
    random = np.random.default_rng(seed)
    means = []
    for _ in range(rounds):
        pick = random.integers(0, len(groups), len(groups))
        sample = np.concatenate([groups[k] for k in pick])
        means.append(sample.mean())
    return float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def rms_from_frames(frames):
    return float(np.sqrt(np.mean(frames)))


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
    ap.add_argument('run', type=Path, help='warm-up recording')
    ap.add_argument('--window', default='25:49')
    ap.add_argument('--grips', type=Path, required=True, help='grip_point_search result.json for this warm-up')
    ap.add_argument('--validate', nargs='*', default=[], help='held-out run_dir:window pairs')
    ap.add_argument('--config', default=str(REPO / 'experiments/hardware/cell/twin.yaml'))
    ap.add_argument('--meshes', default='14,20,28,40')
    ap.add_argument('--scan', default='-1.5,-1,-0.5,0,0.5,1,1.5', help='log10 scales tried for bending and shear')
    ap.add_argument('--resolution-tol-mm', type=float, default=.5)
    ap.add_argument('--detect-tol-mm', type=float, default=1., help='lower bound; raised to 3 x the measured noise sd')
    ap.add_argument('--repeats', type=int, default=4, help='repeated baseline evaluations for the noise level')
    ap.add_argument('--block-sec', type=float, default=2.)
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    begun = time.perf_counter()
    meshes = [int(x) for x in a.meshes.split(',')]
    fine = max(meshes)
    scan = [float(x) for x in a.scan.split(',')]
    fit = (str(a.run.resolve()), a.window)
    windows = {'warm-up (fit)': fit}
    for item in a.validate:   # run:start:end (seconds) or run:warmup (the warm-up the twin announced in that run)
        run, window = (item.rsplit(':', 1) if item.endswith(':warmup') else
                       (item.rsplit(':', 2)[0], ':'.join(item.rsplit(':', 2)[1:])))
        windows[f'held out {Path(run).name} {window}'] = (str(Path(run).resolve()), window)
    pool = mp.get_context('spawn').Pool(safe_workers(a.workers))
    log = open(a.out/'log.jsonl', 'a')
    # Every result is kept in cache.jsonl, keyed by mesh, recording, window and parameters, so a rerun (after a
    # crash, or with more windows) only computes what is missing. Rows from an earlier log.jsonl have no per-frame
    # data; they serve stages that need only the objective (resolution and scans).
    cache, cache_path = {}, a.out/'cache.jsonl'
    for path in (a.out/'log.jsonl', cache_path):
        if path.exists():
            for line in open(path):
                row = json.loads(line)
                if 'key' in row:
                    cache[row['key']] = row['result']
                elif 'stage' in row and 'rms_tanh50_mm' in row:
                    t = dict(cells=row['cells'], run=row['run'], window=row['window'], deployed=row.get('deployed', False),
                             theta=row['theta'])
                    cache.setdefault(task_key(t), {k: v for k, v in row.items() if k not in ('stage', 'cells', 'run', 'window', 'theta', 'deployed')})
    cache_file = open(cache_path, 'a')

    def run(tasks, label, frames=True):
        tasks = [dict(config=a.config, deployed=t.get('deployed', False), **{k: v for k, v in t.items() if k != 'deployed'}) for t in tasks]
        keys = [task_key(t) for t in tasks]
        missing = {}
        for k, t in zip(keys, tasks):
            hit = cache.get(k)
            if hit is None or (frames and 'frame_ms_mm2' not in hit):
                missing.setdefault(k, t)
        groups = {}
        for k, t in missing.items():
            # Repeats go to separate groups, hence separate worker processes: within one process a repeat is
            # bit-identical; the run-to-run noise appears between processes sharing the GPU.
            groups.setdefault((t['cells'], t['run'], t['window'], t['deployed'], t['theta'].get('_repeat')), []).append((k, t))
        if groups:
            print(f'{label}: {len(missing)} new evaluations in {len(groups)} groups ({len(tasks)-len(missing)} cached or repeated)', flush=True)
            done = pool.map(_group, [[t for _, t in g] for g in groups.values()], chunksize=1)
            for g, results in zip(groups.values(), done):
                for (k, _), r in zip(g, results):
                    cache[k] = r
                    cache_file.write(json.dumps(dict(key=k, result=r))+'\n')
            cache_file.flush()
        out = [cache[k] for k in keys]
        for t, r in zip(tasks, out):
            log.write(json.dumps(dict(stage=label, cells=t['cells'], window=t['window'], run=Path(t['run']).name,
                                      theta={k: v for k, v in t['theta'].items() if v}, deployed=t['deployed'],
                                      **{k: v for k, v in r.items() if not k.startswith('frame_')}))+'\n')
        log.flush()
        return out

    # ---- detection on the warm-up, one factor at a time
    grip_values = {k: float(v) for k, v in json.loads(a.grips.read_text())['calibrated'].items() if k in GRIP_NAMES}
    # Repeatability: the same configuration evaluated several times (float32 sums on the GPU differ slightly and the
    # sheet's dynamics amplify them). On 2026-11-20 the baseline scored 27.05 and 27.33 mm in two runs. Detection
    # thresholds are set from this, not chosen.
    reps = run([dict(cells=fine, run=fit[0], window=fit[1], theta={'_repeat': float(i+1)}) for i in range(a.repeats)],
               'repeatability', frames=False)
    rep_values = [objective(r) for r in reps]
    noise_sd = float(np.std(rep_values, ddof=1)) if len(rep_values) > 1 else 0.
    detect_tol = max(a.detect_tol_mm, 3*noise_sd)
    print(f'repeatability: baseline {", ".join(f"{v:.2f}" for v in rep_values)} mm, sd {noise_sd:.2f} mm -> detection threshold '
          f'{detect_tol:.2f} mm', flush=True)

    res = run([dict(cells=m, run=fit[0], window=fit[1], theta={}) for m in meshes], 'resolution', frames=False)
    obj = {m: objective(r) for m, r in zip(meshes, res)}
    best = min(obj.values())
    required = min(m for m in meshes if obj[m] <= best+max(a.resolution_tol_mm, detect_tol))
    print(f'resolution: ' + ', '.join(f'{m}x{m} {obj[m]:.2f}' for m in meshes) + f' mm -> required {required}x{required}', flush=True)

    def scan_factor(name, base, cells):
        out = run([dict(cells=cells, run=fit[0], window=fit[1], theta={**base, name: v}) for v in scan], f'scan-{name}', frames=False)
        values = [objective(r) for r in out]
        k = int(np.argmin(values))
        baseline = values[scan.index(0.)] if 0. in scan else min(values)
        # identifiable: the scan moves the objective beyond the noise and has an interior minimum;
        # adopted: the best value beats the baseline (scale x1) by more than the threshold.
        identifiable = (max(values)-min(values) > detect_tol) and 0 < k < len(scan)-1
        better = baseline-values[k] > detect_tol
        return (dict(scan=scan, objective_mm=values, best=scan[k], identifiable=bool(identifiable), gain_mm=baseline-values[k],
                     threshold_mm=detect_tol, adopted=bool(better)), (scan[k] if better else 0.))

    bend_scan, bend = scan_factor('bend', {}, fine)
    shear_scan, shear = scan_factor('shear', {}, fine)
    print(f'bending: best x{10**bend_scan["best"]:.3g} (gain {bend_scan["gain_mm"]:.2f} mm, identifiable {bend_scan["identifiable"]}, '
          f'adopted {bend_scan["adopted"]}); shear: best x{10**shear_scan["best"]:.3g} (gain {shear_scan["gain_mm"]:.2f} mm, '
          f'identifiable {shear_scan["identifiable"]}, adopted {shear_scan["adopted"]})', flush=True)
    joint_shear_scan, joint_shear = scan_factor('shear', grip_values, fine)
    joint_bend_scan, joint_bend = scan_factor('bend', {**grip_values, 'shear': joint_shear}, fine)
    print(f'with grip points: shear best x{10**joint_shear_scan["best"]:.3g} (gain {joint_shear_scan["gain_mm"]:.2f} mm, adopted '
          f'{joint_shear_scan["adopted"]}), bending best x{10**joint_bend_scan["best"]:.3g} (gain {joint_bend_scan["gain_mm"]:.2f} mm, '
          f'adopted {joint_bend_scan["adopted"]})', flush=True)

    # ---- factorial + joint + deployed reference, on every window
    factors = dict(R=(fine, required if required != fine else 28), G=({}, grip_values), B=(0., bend), S=(0., shear))
    configs = []
    for levels in itertools.product((0, 1), repeat=4):
        r, g, b, s = levels
        theta = {**factors['G'][g], **({'bend': factors['B'][b]} if b else {}), **({'shear': factors['S'][s]} if s else {})}
        configs.append(dict(name=''.join(f for f, on in zip('RGBS', levels) if on) or 'baseline', levels=levels,
                            cells=factors['R'][r], theta=theta))
    for cells in sorted({fine, required}):
        configs.append(dict(name=f'joint@{cells}', levels=None, cells=cells,
                            theta={**grip_values, 'bend': joint_bend, 'shear': joint_shear}))
    tasks, keys = [], []
    for c in configs:
        for wname, (wrun, wwin) in windows.items():
            tasks.append(dict(cells=c['cells'], run=wrun, window=wwin, theta=c['theta'])); keys.append((c['name'], wname))
    results = dict(zip(keys, run(tasks, 'evaluation')))
    deployed = run([dict(cells=28, run=wr, window=ww, deployed=True, theta={}) for wr, ww in windows.values()], 'deployed')
    for wname, r in zip(windows, deployed):
        results[('deployed', wname)] = r
    # Bending and shear on the held-out windows, one decade either side of the detected value, with and without the
    # grip points, at the required mesh: tests each in isolation where the factorial cannot (a detected value equal
    # to the baseline makes its factorial levels identical).
    heldout_scan = {}
    held_windows = {w: v for w, v in windows.items() if w != 'warm-up (fit)'}
    for grips_on in (False, True):
        base_theta = dict(grip_values) if grips_on else {}
        for name, centre in (('bend', joint_bend if grips_on else bend), ('shear', joint_shear if grips_on else shear)):
            values = [centre-1., centre, centre+1.]
            items = [(v, wname, wr, ww) for v in values for wname, (wr, ww) in held_windows.items()]
            out = run([dict(cells=required, run=wr, window=ww, theta={**base_theta, name: v}) for v, _, wr, ww in items],
                      f'heldout-scan-{name}{"-grips" if grips_on else ""}', frames=False)
            for (v, wname, _, _), r in zip(items, out):
                heldout_scan.setdefault(f'{name}{" with grip points" if grips_on else ""}', {}).setdefault(wname, {})[f'{v:+.2f}'] = \
                    dict(objective_mm=objective(r), median_mm=r['median_mm'], p95_abs_mm=r['p95_abs_mm'])
    pool.close(); pool.join()

    # ---- effects
    effects = {}
    for wname in windows:
        effects[wname] = {}
        for i, f in enumerate('RGBS'):
            on = [c for c in configs if c['levels'] and c['levels'][i] == 1]
            diffs = []
            for c in on:
                off_levels = list(c['levels']); off_levels[i] = 0
                off = next(x for x in configs if x['levels'] == tuple(off_levels))
                diffs.append(np.array(results[(c['name'], wname)]['frame_ms_mm2'])-np.array(results[(off['name'], wname)]['frame_ms_mm2']))
            diff = np.mean(diffs, axis=0)
            t = results[('baseline', wname)]['frame_t']
            lo, hi = block_bootstrap(diff, t, a.block_sec)
            alone = results[(f, wname)]
            degenerate = factors[f][0] == factors[f][1]
            effects[wname][f] = dict(main_effect_ms_mm2=float(diff.mean()), ci95=[lo, hi],
                                     alone_minus_baseline_mm=objective(alone)-objective(results[('baseline', wname)]),
                                     tested=not degenerate,
                                     note='detected value equals the baseline: nothing to adopt' if degenerate else '')
    held = [w for w in windows if w != 'warm-up (fit)']
    # Adoption, on held-out windows only. G, B, S: no window significantly worse, at least one significantly
    # better, and better on average. R (the coarser mesh): adopted when no window is significantly worse, since
    # its purpose is speed at no loss.
    def harm(w, f):
        return effects[w][f]['ci95'][0] > 0

    def benefit(w, f):
        return effects[w][f]['ci95'][1] < 0
    adopted = {}
    for f in 'RGBS':
        mean = np.mean([effects[w][f]['main_effect_ms_mm2'] for w in held]) if held else np.nan
        if factors[f][0] == factors[f][1]:
            adopted[f] = False
        elif f == 'R':
            adopted[f] = bool(held) and not any(harm(w, f) for w in held)
        else:
            adopted[f] = bool(held) and not any(harm(w, f) for w in held) and any(benefit(w, f) for w in held) and mean < 0

    # ---- outputs
    names = [c['name'] for c in configs]+['deployed']
    with open(a.out/'table.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['config', 'mesh', 'window', 'objective_mm', 'median_mm', 'p95_abs_mm', 'over_30mm'])
        for n in names:
            cells = 28 if n == 'deployed' else next(c['cells'] for c in configs if c['name'] == n)
            for wname in windows:
                r = results[(n, wname)]
                writer.writerow([n, cells, wname, round(objective(r), 2), round(r['median_mm'], 1), round(r['p95_abs_mm'], 1),
                                 round(r['over_30mm'], 3)])
    detected = dict(repeatability=dict(objective_mm=rep_values, sd_mm=noise_sd, threshold_mm=detect_tol),
                    resolution=dict(objective_mm=obj, required=required), bend=bend_scan, shear=shear_scan,
                    joint=dict(shear=joint_shear_scan, bend=joint_bend_scan), grips=grip_values)
    result = dict(warmup=fit, windows=windows, detected=detected, factors={k: (v if k != 'G' else ['measured', 'calibrated'])
                                                                           for k, v in factors.items()},
                  configs=[dict(c, levels=list(c['levels']) if c['levels'] else None) for c in configs],
                  results={f'{k[0]} | {k[1]}': {kk: vv for kk, vv in v.items() if not kk.startswith('frame_')}
                           for k, v in results.items()},
                  effects=effects, adopted=adopted, heldout_scan=heldout_scan, seconds=time.perf_counter()-begun)
    plain = lambda o: o.item() if hasattr(o, 'item') else str(o)  # numpy scalars and bools  # noqa: E731
    (a.out/'result.json').write_text(json.dumps(result, indent=1, default=plain))
    lines = ['# Warm-up calibration and factor test', '',
             f'Warm-up {fit[0]} {fit[1]}; held out: ' + '; '.join(held), '',
             f'Repeatability: baseline sd {noise_sd:.2f} mm over {len(rep_values)} runs; detection threshold {detect_tol:.2f} mm.', '',
             f'Detected: required mesh {required}x{required}; bending best x{10**bend_scan["best"]:.3g} (gain {bend_scan["gain_mm"]:.2f} mm, '
             f'adopted {bend_scan["adopted"]}); shear best x{10**shear_scan["best"]:.3g} (gain {shear_scan["gain_mm"]:.2f} mm, adopted '
             f'{shear_scan["adopted"]}); with grip points shear x{10**joint_shear:.3g}, bending x{10**joint_bend:.3g}.', '',
             '| Config | ' + ' | '.join(windows) + ' |', '| --- |' + ' --- |'*len(windows)]
    for n in names:
        lines.append(f'| {n} | ' + ' | '.join(f'{objective(results[(n, w)]):.1f} ({results[(n, w)]["median_mm"]:+.0f}, '
                                              f'{results[(n, w)]["p95_abs_mm"]:.0f})' for w in windows) + ' |')
    lines += ['', 'Cells: objective mm (median mm, p95 mm). Main effects (mm^2 of mean square per frame, 95 % block-bootstrap interval):', '']
    for wname in windows:
        lines.append(f'- {wname}: ' + '; '.join(f'{f} {e["main_effect_ms_mm2"]:+.1f} [{e["ci95"][0]:+.1f}, {e["ci95"][1]:+.1f}]'
                                               for f, e in effects[wname].items()))
    lines += ['', 'Bending and shear one decade either side of the detected value on the held-out windows (objective mm):', '']
    for name, per_window in heldout_scan.items():
        for wname, values in per_window.items():
            lines.append(f'- {name}, {wname}: ' + ', '.join(f'{k} {v["objective_mm"]:.2f}' for k, v in values.items()))
    lines += ['', 'Adopted (held-out evidence): ' + ', '.join(f'{f}={v}' for f, v in adopted.items()),
              '', 'R on = the coarser mesh instead of the finest; adopting R means the coarser mesh costs nothing measurable.']
    (a.out/'report.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines), flush=True)


if __name__ == '__main__':
    main()
