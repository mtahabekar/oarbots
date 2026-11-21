#!/usr/bin/env python3
"""Which warm-up motion can tell bending, shear and the grip points apart? (`fabric_run.sh warmup-design --out <dir>`)

Before running a new warm-up on the robots, simulate it. For each candidate profile (keyword sets of
fabric_twin.warmup.WarmupScript, as in twin.yaml `warmup:`) the robot grips follow the profile from the warm-up
start corners of a recorded run while the hands stay where they were. A "truth" sheet (the cell's twin: mesh,
solver, measured material, point grips) is sampled as depth points on its surface, 128 per frame at 5 Hz. The
forward model of the calibration (fabric_twin.identify.Forward, scored on those points exactly as on camera depth)
is then run with

    stretch, shear, bend   -1, -0.5, +0.5, +1 decade
    down_<c>, in_<c>        +1 cm, for the four corners (the grip-point nuisance the calibration also fits)

Reported per candidate:
    effect_mm        RMS change of the depth residual for each material change
    unique_mm        what is left of it after the best compensation by the eight grip offsets (least squares):
                     the part of the material's signature a grip-point error cannot imitate
    detectable       unique_mm above the detection limit of the calibration on hardware: with model error
                     e = 20 mm RMS left after calibration and the 1 mm objective threshold of warmup_calibrate.py,
                     a change is seen when sqrt(e^2 + u^2) - e > 1 mm, i.e. u > 6.4 mm
    cramer_rao       joint Cramér-Rao sd (decades for material, cm for offsets) with 8 cm per point at 64
                     points per frame, the noise ES-MDA uses on hardware; only for comparing candidates
    geometry         duration, largest displacement, distance of every robot target to the bridge workspace box
                     faces (bridge.yaml), and the largest grip-pair distance relative to the start

A recorded run can be added as an anchor ("live:<run_dir>:<start>:<end>", robots and hands as recorded), so the
candidates can be compared with motion the hardware data already showed something about.
GPU; no ROS graph, no robots. Start CUDA MPS first (fabric_run.sh mps start).
"""
import argparse
import dataclasses
import gc
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/fabric_twin'))
sys.path.insert(0, str(REPO / 'scripts/hw'))

CORNERS = ('blue', 'silver', 'right', 'left')
MATERIAL = ('stretch', 'shear', 'bend')
NUISANCE = tuple(f'{k}_{c}' for k in ('down', 'in') for c in CORNERS)
NAMES = MATERIAL+NUISANCE
DECADES = (-1., -.5, .5, 1.)
OFFSET_M = .01
FRAME_HZ = 60
DEPTH_EVERY = 12            # 5 Hz
POINTS = 128
MODEL_ERROR_M = .02         # RMS depth error left after calibration (2026-11-20 warm-up: 19.5-20.4 mm)
THRESHOLD_M = .001          # warmup_calibrate.py detection threshold (objective)
DETECT_M = float(np.sqrt((MODEL_ERROR_M+THRESHOLD_M)**2-MODEL_ERROR_M**2))

# Candidates: the deployed 2-4 cm insets, each extra excursion alone (one change at a time), and combinations.
CANDIDATES = {
    'current': dict(insets_m=[.02, .04]),
    'deep': dict(insets_m=[.04, .08]),
    'shear': dict(insets_m=[.02, .04], lateral_m=.08),
    'lift': dict(insets_m=[.02, .04], lift_m=.06),
    'narrow': dict(insets_m=[.02, .04], transverse_m=.08),
    'deep+shear': dict(insets_m=[.04, .08], lateral_m=.08),
    'all': dict(insets_m=[.04, .08], transverse_m=.08, lift_m=.06, lateral_m=.08),
    # The person moves the hands on cues while the robots hold (after the deployed 2-4 cm insets); one move at a time,
    # then all of them, then all of them with the robot excursions.
    'hands up/down': dict(insets_m=[.02, .04], person_moves_m=dict(up=.15, down=.15)),
    'hands toward': dict(insets_m=[.02, .04], person_moves_m=dict(toward=.15)),
    'hands together': dict(insets_m=[.02, .04], person_moves_m=dict(toward=.15, together=.12)),
    'hands side': dict(insets_m=[.02, .04], person_moves_m=dict(toward=.15, side=.10)),
    'hands all': dict(insets_m=[.02, .04], person_moves_m=dict(up=.15, down=.15, toward=.15, together=.12, side=.10)),
    # Is it the size or the speed of the person's moves? (the recorded carry moved the hands 45-70 cm at up to 40 cm/s)
    'hands large': dict(insets_m=[.02, .04], person_moves_m=dict(up=.25, down=.25, toward=.25, together=.2, side=.15)),
    'hands fast': dict(insets_m=[.02, .04], person_moves_m=dict(up=.15, down=.15, toward=.15, together=.12, side=.10),
                       person_speed_mps=.3, person_reaction_sec=0., hold_sec=1.5),
    'robots all + hands all': dict(insets_m=[.04, .08], transverse_m=.08, lift_m=.06, lateral_m=.08,
                                   person_moves_m=dict(up=.15, down=.15, toward=.15, together=.12, side=.10)),
}

_cache = {}


def spec_from(config):
    from fabric_twin.cloth import Obstacle
    from fabric_twin.node import load_twin_config
    spec = load_twin_config(Path(config))[0]
    return dataclasses.replace(spec, grip_insets_m=(0., 0., 0., 0.),
                               obstacle=Obstacle(center=(0., 0., -50.), half_extents=(.1, .1, .1)))


def profile(start, kwargs, hold_sec, speed):
    """Grip frames (T, 4, 3) at 60 Hz: robots on the profile (ideal tracking), hands still or on the person cues."""
    from fabric_twin.warmup import WarmupScript
    script = WarmupScript(**{'hold_sec': hold_sec, 'speed_mps': speed, **kwargs})
    script.start(0, start)
    t = np.arange(int(script.duration*FRAME_HZ)+1)/FRAME_HZ
    grips = np.repeat(start[None], len(t), axis=0)
    for k, at in enumerate(t):
        grips[k, :2] += script.offset(at)
        grips[k, 2:] += script.hand_offset(at)       # person cues (the person is assumed to follow them)
    return grips, script


def sample_surface(q, grid, count, random, margin=.04):
    """Points on the sheet (bilinear in the mesh), away from the edges where hands and grippers occlude it."""
    s = q[grid]                                          # (ny+1, nx+1, 3)
    ny, nx = grid.shape[0]-1, grid.shape[1]-1
    u, v = random.uniform(margin, 1-margin, (2, count))
    x, y = u*nx, v*ny
    i, j = np.minimum(x.astype(int), nx-1), np.minimum(y.astype(int), ny-1)
    a, b = (x-i)[:, None], (y-j)[:, None]
    return (s[j, i]*(1-a)*(1-b)+s[j, i+1]*a*(1-b)+s[j+1, i]*(1-a)*b+s[j+1, i+1]*a*b)


def _truth(task):
    """Run the truth sheet along the grips; return the window's depth points."""
    from fabric_twin.identify import SheetBatch, grip_spans, grip_uv, within_reach
    spec = spec_from(task['config'])
    grips = np.asarray(task['grips'])
    batch = SheetBatch(spec, 1, grips[0], pins_uv=grip_uv(spec, (0., 0., 0., 0.)))
    spans = grip_spans(spec, batch.pins_uv)
    random = np.random.default_rng(task.get('seed', 0))
    first = within_reach(grips[:1], spec.width_m, spec.height_m, spans=spans)
    batch.place(first, .03)
    for _ in range(120):
        batch.step(first)
    index, points = [], []
    for k in range(len(grips)):
        batch.step(within_reach(grips[k:k+1], spec.width_m, spec.height_m, spans=spans))
        if k >= FRAME_HZ and k % DEPTH_EVERY == 0:      # skip the first second, as make_window does
            index.append(k); points.append(sample_surface(batch.positions()[0], batch.grid, POINTS, random))
    del batch; gc.collect()
    return dict(name=task['name'], depth_index=index, points=[p.tolist() for p in points])


def _row(task):
    from fabric_twin.identify import Forward, Window
    key = task['name']
    if key not in _cache:
        _cache.clear(); gc.collect()
        w = task['window']
        window = Window(np.arange(len(w['grips']))*int(1e9/FRAME_HZ), np.asarray(w['grips']),
                        np.asarray(w['depth_index']), [np.asarray(p) for p in w['points']])
        _cache[key] = Forward(spec_from(task['config']), window, list(NAMES), point_grips=True)
    theta = np.array([task['theta'].get(n, 0.) for n in NAMES])
    raw = _cache[key].raw(theta[None])[0]
    return dict(name=key, row=task['row'], raw=raw.astype(np.float32))


def rows():
    out = [('nominal', {})]
    out += [(f'{m}{d:+.1f}', {m: d}) for m in MATERIAL for d in DECADES]
    out += [(f'{n}+1cm', {n: OFFSET_M}) for n in NUISANCE]
    return out


def analyse(raw, names):
    """raw: dict row-name -> residual (M,) in m. Material effects, unique effects and Cramér-Rao sd."""
    def bounded(x):
        return np.where(np.isfinite(x), .1*np.tanh(np.nan_to_num(x)/.1), .1)
    y0 = bounded(raw['nominal'])
    nuisance = np.column_stack([(bounded(raw[f'{n}+1cm'])-y0)/OFFSET_M for n in NUISANCE])     # per m
    out = dict(nominal_rms_mm=float(np.sqrt(np.mean(y0**2))*1e3), material={})
    for m in MATERIAL:
        cells = {}
        for d in DECADES:
            delta = bounded(raw[f'{m}{d:+.1f}'])-y0
            coef, *_ = np.linalg.lstsq(nuisance, delta, rcond=None)
            left = delta-nuisance @ coef
            cells[f'{d:+.1f}'] = dict(effect_mm=float(np.sqrt(np.mean(delta**2))*1e3),
                                      unique_mm=float(np.sqrt(np.mean(left**2))*1e3),
                                      detectable=bool(np.sqrt(np.mean(left**2)) > DETECT_M),
                                      compensating_offsets_cm={n: round(float(c)*100, 2) for n, c in zip(NUISANCE, coef)})
        out['material'][m] = cells
    # Joint Fisher information: material by central differences over +-0.5 decade, offsets by +1 cm.
    J = np.column_stack([(bounded(raw[f'{m}+0.5'])-bounded(raw[f'{m}-0.5']))/1. for m in MATERIAL]+[nuisance*.01])
    frames = len(y0)/POINTS
    noise = .08
    fisher = J.T @ J/noise**2*(64/POINTS)          # same information per frame as 64 points at 8 cm
    covariance = np.linalg.pinv(fisher)
    sd = np.sqrt(np.maximum(np.diag(covariance), 0.))
    corr = covariance/np.maximum(np.outer(sd, sd), 1e-300)
    out['cramer_rao'] = dict(units='decades for material, cm for offsets', noise_m=noise, frames=frames,
                             sd={n: float(s) for n, s in zip(NAMES, sd)},
                             largest_correlation={n: float(np.max(np.abs(np.delete(corr[i], i)))) for i, n in enumerate(NAMES)})
    out['nuisance_effect_mm_per_cm'] = {n: float(np.sqrt(np.mean((nuisance[:, i]*OFFSET_M)**2))*1e3) for i, n in enumerate(NUISANCE)}
    return out


def geometry(start, grips, script, bridge):
    """Workspace-box margins of every robot grip along the profile, and pair distances against the start."""
    boxes = {r: (np.array({**bridge['common'], **bridge[r]}['workspace_min']),
                 np.array({**bridge['common'], **bridge[r]}['workspace_max'])) for r in ('blue', 'silver')}
    margin = {}
    for i, r in enumerate(('blue', 'silver')):
        lo, hi = boxes[r]
        p = grips[:, i]
        margin[r] = float(min(np.min(p-lo), np.min(hi-p)))
    pairs = [(0, 1), (2, 3), (0, 2), (1, 3), (0, 3), (1, 2)]
    d0 = {f'{a}-{b}': float(np.linalg.norm(start[a]-start[b])) for a, b in pairs}
    growth = {f'{a}-{b}': float(np.max(np.linalg.norm(grips[:, a]-grips[:, b], axis=1))-d0[f'{a}-{b}']) for a, b in pairs}
    shift = grips[:, :2]-start[None, :2]
    return dict(duration_s=float(script.duration), holds=len(script.holds()),
                largest_displacement_cm=float(np.max(np.linalg.norm(shift, axis=2))*100),
                workspace_margin_cm={r: round(v*100, 1) for r, v in margin.items()},
                pair_growth_cm={k: round(v*100, 2) for k, v in growth.items()}, start_pair_m=d0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reference', type=Path,
                    default=REPO/'experiments/hardware/sessions/2026-11-20_fabric_run/warmup_10_0.1_111137',
                    help='run whose warm-up start corners are used')
    ap.add_argument('--candidates', nargs='*', default=list(CANDIDATES), help='names from CANDIDATES, or name=JSON kwargs')
    ap.add_argument('--anchors', nargs='*', default=[], help='live:<run_dir>:<start>:<end> recorded motion')
    ap.add_argument('--hold-sec', type=float, default=5.)
    ap.add_argument('--speed', type=float, default=.03)
    ap.add_argument('--config', default=str(REPO/'experiments/hardware/cell/twin.yaml'))
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--per-worker-gb', type=float, default=2.,
                    help='host memory per worker for the cap (14 x 14 workers use ~0.5 GB; 40 x 40 grip searches 5.7)')
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    import identify_replay as ir
    import warmup_calibrate as wc
    a.out.mkdir(parents=True, exist_ok=True)
    bridge = yaml.safe_load((REPO/'experiments/hardware/cell/bridge.yaml').read_text())
    ref = ir.load(a.reference.resolve(), 'warmup', 2, 64, 1.)
    start = ref.grips[0]
    cases = {}
    for c in a.candidates:
        name, kwargs = (c.split('=', 1)[0], json.loads(c.split('=', 1)[1])) if '=' in c else (c, CANDIDATES[c])
        grips, script = profile(start, kwargs, a.hold_sec, a.speed)
        cases[name] = dict(kwargs=kwargs, grips=grips, geometry=geometry(start, grips, script, bridge))
    for anchor in a.anchors:
        parts = anchor.split(':')
        run, s, e = ':'.join(parts[1:-2]), parts[-2], parts[-1]
        w = ir.load(Path(run).resolve(), f'{s}:{e}', 2, 64, 1.)      # the twin's 60 Hz grip records
        name = f'recorded {Path(run).name} {s}:{e}'
        cases[name] = dict(kwargs=None, grips=w.grips, geometry=dict(duration_s=float((w.frame_ns[-1]-w.frame_ns[0])/1e9)))
    for name, case in cases.items():
        print(f'{name:>24}: {case["geometry"]}', flush=True)
    begun = time.perf_counter()
    workers = wc.safe_workers(a.workers, per_worker_gb=a.per_worker_gb)
    with mp.get_context('spawn').Pool(workers) as pool:
        truths = pool.map(_truth, [dict(name=n, config=a.config, grips=c['grips'].tolist()) for n, c in cases.items()], chunksize=1)
        print(f'truth sheets done in {time.perf_counter()-begun:.0f} s', flush=True)
        tasks = []
        for t in truths:
            window = dict(grips=cases[t['name']]['grips'].tolist(), depth_index=t['depth_index'], points=t['points'])
            for row, theta in rows():
                tasks.append(dict(name=t['name'], config=a.config, window=window, row=row, theta=theta))
        results = {}
        for r in pool.imap(_row, tasks, chunksize=1):
            results.setdefault(r['name'], {})[r['row']] = r['raw']
            done = sum(len(v) for v in results.values())
            if done % 10 == 0:
                print(f'{done}/{len(tasks)} forward runs, {time.perf_counter()-begun:.0f} s', flush=True)
    report = dict(reference=str(a.reference), detect_mm=DETECT_M*1e3, model_error_mm=MODEL_ERROR_M*1e3,
                  hold_sec=a.hold_sec, speed_mps=a.speed, candidates={})
    for name, case in cases.items():
        report['candidates'][name] = dict(kwargs=case['kwargs'], geometry=case['geometry'], **analyse(results[name], NAMES))
    (a.out/'result.json').write_text(json.dumps(report, indent=1))
    lines = [f'# Warm-up design study ({time.strftime("%F %H:%M")})', '',
             f'Start corners from {a.reference.name} (warm-up start). Truth = the cell twin ({Path(a.config).name}), '
             f'depth sampled at 5 Hz, {POINTS} points per frame. Detection limit {DETECT_M*1e3:.1f} mm unique effect '
             f'(model error {MODEL_ERROR_M*1e3:.0f} mm, threshold {THRESHOLD_M*1e3:.0f} mm).', '',
             '| candidate | s | shear -1 | shear -0.5 | shear +0.5 | shear +1 | bend -1 | bend +1 | stretch -1 | CR sd shear | CR sd bend |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for name, r in report['candidates'].items():
        m = r['material']
        def cell(p, d):
            c = m[p][d]
            return f'{c["effect_mm"]:.1f} / {c["unique_mm"]:.1f}{" *" if c["detectable"] else ""}'
        lines.append(f'| {name} | {r["geometry"]["duration_s"]:.0f} | {cell("shear", "-1.0")} | {cell("shear", "-0.5")} | '
                     f'{cell("shear", "+0.5")} | {cell("shear", "+1.0")} | {cell("bend", "-1.0")} | {cell("bend", "+1.0")} | '
                     f'{cell("stretch", "-1.0")} | {r["cramer_rao"]["sd"]["shear"]:.3f} | {r["cramer_rao"]["sd"]["bend"]:.3f} |')
    lines += ['', 'Cells: effect / unique (mm RMS of the depth residual); * = above the detection limit.', '']
    for name, r in report['candidates'].items():
        lines.append(f'- {name}: geometry {json.dumps(r["geometry"].get("workspace_margin_cm", {}))} cm to the box faces, '
                     f'pair growth {json.dumps(r["geometry"].get("pair_growth_cm", {}))} cm; grip offsets 1 cm -> '
                     + ', '.join(f'{k} {v:.1f}' for k, v in r['nuisance_effect_mm_per_cm'].items()) + ' mm; CR sd offsets '
                     + ', '.join(f'{k} {r["cramer_rao"]["sd"][k]:.2f}' for k in NUISANCE) + ' cm')
    (a.out/'report.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines), flush=True)


if __name__ == '__main__':
    main()
