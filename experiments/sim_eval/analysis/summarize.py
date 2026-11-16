"""Collect every study's results into tables (results/*.csv) for the report and the figures.

    python summarize.py s1|s2|s3|s4|all

s1  hand tracking and estimator accuracy per dataset and phase, scored against plant truth
s2  identification fits                     (results/s2/*.json)
s3  twin replays, state and forecast error  (results/s3/*.npz)
s4  closed-loop carries                     ($SIM_EVAL_DATA/closed/*/score.json)
"""
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RESULTS = HERE.parent/'results'
DATA = Path(os.environ.get('SIM_EVAL_DATA', '/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval'))
PHASES = {'warmup': (15.0, 93.33), 'validation': (97.5, 140.4)}
HORIZONS = (6, 20, 30, 60)


def write(name, rows):
    if not rows:
        print(f'{name}: no rows'); return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(RESULTS/name, 'w', newline='') as f:
        w = csv.DictWriter(f, keys); w.writeheader(); w.writerows(rows)
    print(f'{name}: {len(rows)} rows')


def datasets():
    return sorted(p for p in (DATA/'id').glob('id_*') if (p/'manifest.json').exists())


def ident(run, manifest):
    return dict(dataset=run.name, structure=manifest['structure'], material=manifest['material'], repeat=manifest['repeat'],
                true_bend=manifest['true_scales']['bend'], true_shear=manifest['true_scales']['shear'])


# ------------------------------------------------------------------ S1
def estimator_errors(run, e, base, extra):
    """Estimator analyses against plant truth at their measurement times, by phase (interior vertices)."""
    rows = []
    u, v = np.meshgrid(np.linspace(0, 1, 8), np.linspace(0, 1, 8))
    truth = run.truth_at_uv(np.column_stack((u.ravel(), v.ravel())))
    analyses = np.flatnonzero(~e['predicted'])
    interior = np.ones(64, bool); interior[[0, 7, 56, 63]] = False
    for phase, (lo, hi) in PHASES.items():
        idx = analyses[(e['measurement_ns'][analyses] >= lo*1e9) & (e['measurement_ns'][analyses] <= hi*1e9)]
        if not len(idx):
            continue
        k = np.array([run.frame(s) for s in e['measurement_ns'][idx]])
        dist = np.linalg.norm(e['positions'][idx]-truth[k], axis=2)[:, interior]
        sigma = np.sqrt(np.trace(e['covariance'][idx][:, :, :3, :3], axis1=2, axis2=3))[:, interior]
        rows.append(dict(base, **extra, phase=phase, analyses=len(idx), analysis_rate_hz=len(idx)/(hi-lo),
                         est_error_p50_mm=1e3*np.percentile(dist, 50), est_error_p95_mm=1e3*np.percentile(dist, 95),
                         est_error_mean_mm=1e3*dist.mean(),
                         # fraction inside the stated 95 % sphere (chi, 3 dof: 2.796 per-axis sigma)
                         coverage95=float((dist <= 2.796*sigma/np.sqrt(3)).mean()),
                         degraded_frac=float(e['degraded'][idx].mean()),
                         observed_frac=float((e['visibility'][idx][:, interior] == 1).mean())))
    return rows


def s1b():
    """Observer replays: vision on/off and oracle hands on the same recorded inputs."""
    from simeval import Run
    rows, cache = [], {}
    for path in sorted((DATA/'obs').glob('*/estimate.npz')):
        dataset, condition = path.parent.name.split('.')
        if dataset not in cache:
            cache.clear(); cache[dataset] = Run(DATA/'id'/dataset)
        run = cache[dataset]
        with np.load(path) as z:
            e = {k: z[k] for k in z.files}
        rows += estimator_errors(run, e, ident(run.path, run.manifest), dict(condition=condition))
    write('s1b_observer.csv', rows)


def s1():
    from simeval import Run
    hand_rows, est_rows = [], []
    for path in datasets():
        run = Run(path)
        base = ident(path, run.manifest)
        hands = np.load(path/'estimate.hands.npy')
        for phase, (lo, hi) in PHASES.items():
            row = dict(base, phase=phase)
            errors, latency, gaps = [], [], []
            for h in (0, 1):
                rows = hands[hands[:, 1] == h]
                t = rows[:, 0]
                keep = (t >= lo*1e9) & (t <= hi*1e9)
                truth = run.grips('truth', t[keep].astype(np.int64))[:, 2+h]
                errors.append(np.linalg.norm(rows[keep, 2:5]-truth, axis=1))
                latency.append((rows[keep, 7]-rows[keep, 0])/1e9)
                stamps = np.unique(t[keep])/1e9
                gaps.append(np.diff(np.concatenate(([lo], stamps, [hi]))))
            e, lat, g = np.concatenate(errors), np.concatenate(latency), np.concatenate(gaps)
            row.update(hand_error_p50_mm=1e3*np.percentile(e, 50), hand_error_p95_mm=1e3*np.percentile(e, 95),
                       hand_latency_p50_ms=1e3*np.percentile(lat, 50), hand_latency_p95_ms=1e3*np.percentile(lat, 95),
                       hand_rate_hz=len(e)/2/(hi-lo), hand_unseen_frac=float(g[g > .2].sum()/(2*(hi-lo))),
                       hand_longest_gap_s=float(g.max()))
            # the grips the twin was actually stepped with (TF for robots, glove filter carried to the twin's time)
            tw = run.twin['corners_ns']; keep = (tw >= lo*1e9) & (tw <= hi*1e9)
            d = np.linalg.norm(run.twin['corners'][keep]-run.grips('truth', tw[keep]), axis=2)
            row.update(twin_robot_grip_p95_mm=1e3*np.percentile(d[:, :2], 95), twin_hand_grip_p50_mm=1e3*np.percentile(d[:, 2:], 50),
                       twin_hand_grip_p95_mm=1e3*np.percentile(d[:, 2:], 95))
            hand_rows.append(row)
        est_rows += estimator_errors(run, run.estimate, base, dict(condition='live'))
        print(path.name, 'S1 scored', flush=True)
    write('s1_hands.csv', hand_rows)
    write('s1_estimator.csv', est_rows)


# ------------------------------------------------------------------ S2
def s2():
    rows = []
    adapt = []
    for path in sorted((RESULTS/'s2').glob('*.adapt.json')):
        r = json.loads(path.read_text())
        trace = r['trace']
        adapt.append(dict(ident(Path(r['run']), r['manifest']), level=path.name.split('.')[1], inputs='/'.join(r['inputs'].values()),
                          final_bend=r['final']['bend'], final_shear=r['final']['shear'],
                          log_err_bend=r['final']['log_err_bend'], log_err_shear=r['final']['log_err_shear'],
                          updates=len(trace), moves=sum(bool(t.get('moved')) for t in trace),
                          informative_frac=float(np.mean([bool(t.get('informative')) for t in trace if not t.get('skipped')]))))
    write('s2_adapt.csv', adapt)
    for path in sorted((RESULTS/'s2').glob('*.json')):
        if path.name.endswith('.adapt.json'):
            continue
        r = json.loads(path.read_text())
        manifest = r['manifest']
        level = path.name.split('.')[1]
        row = dict(ident(Path(r['run']), manifest), level=level, holds=r['holds'], cases=r['cases'],
                   inputs='/'.join(r['inputs'].values()))
        if 'fit' in r:
            f = r['fit']
            row.update(bend=f['bend'], shear=f['shear'], log_err_bend=r['log_error']['bend'], log_err_shear=r['log_error']['shear'],
                       cost_mm=1e3*f['cost_m'], start_cost_mm=1e3*f['start_cost_m'], sensitivity_mm=1e3*f['sensitivity_m'],
                       evaluations=f['evaluations'], fit_s=r['fit_seconds'])
            # continuous minimum: quadratic fit in log space to the evaluated table near the best point
            table = np.array([(np.log(t['bend']), np.log(t['shear']), t['cost_m']**2) for t in f['table']])
            best = np.array([np.log(f['bend']), np.log(f['shear'])])
            near = table[np.linalg.norm(table[:, :2]-best, axis=1) <= np.log(2.)*1.01]
            if len(near) >= 6:
                x, y, z = near.T
                A = np.column_stack((x*x, y*y, x*y, x, y, np.ones_like(x)))
                c = np.linalg.lstsq(A, z, rcond=None)[0]
                H = np.array([[2*c[0], c[2]], [c[2], 2*c[1]]])
                if np.all(np.linalg.eigvalsh(H) > 0):
                    m = np.linalg.solve(H, -c[3:5])
                    if np.linalg.norm(m-best) < np.log(2.):
                        row.update(bend_refined=float(np.exp(m[0])), shear_refined=float(np.exp(m[1])),
                                   log_err_bend_refined=float(m[0]-np.log(manifest['true_scales']['bend'])),
                                   log_err_shear_refined=float(m[1]-np.log(manifest['true_scales']['shear'])))
        if 'landscape' in r:
            row['landscape'] = True
        rows.append(row)
    write('s2_fits.csv', rows)


# ------------------------------------------------------------------ S3
def s3():
    rows = []
    for path in sorted((RESULTS/'s3').glob('*.npz')):
        z = np.load(path)
        meta = json.loads(str(z['meta']))
        manifest = meta['manifest']
        row = dict(ident(Path(meta['run']), manifest), condition=meta['label'], correction=meta['correction'],
                   twin_bend=meta['scales'][0], twin_shear=meta['scales'][1], grips=meta['grips'],
                   feedback_scale=meta['feedback_scale'], correction_force_p95_n=meta['info']['force_total_n'])
        t = z['t']
        for phase, (lo, hi) in PHASES.items():
            m = (t >= lo) & (t <= hi)
            row[f'{phase}_mean_mm'] = 1e3*z['state_mean'][m].mean()
            row[f'{phase}_p95_mm'] = 1e3*np.percentile(z['state_p95'][m], 95)
            row[f'{phase}_low_abs_dz_mm'] = 1e3*np.abs(z['state_low_dz'][m]).mean()
        for h in HORIZONS:
            row[f'forecast_{h}_mean_mm'] = 1e3*z[f'forecast_{h}_mean'].mean()
            row[f'forecast_{h}_low_abs_dz_mm'] = 1e3*np.abs(z[f'forecast_{h}_low_dz']).mean()
        rows.append(row)
    write('s3_replays.csv', rows)


# ------------------------------------------------------------------ S4
def s4():
    rows = []
    for path in sorted((DATA/'closed').glob('cl_*/score.json')):
        s = json.loads(path.read_text())
        row = {k: v for k, v in s.items() if not isinstance(v, dict)}
        for key in ('pull_robot', 'pull_hands'):
            for k, v in (s.get(key) or {}).items():
                row[f'{key}_{k}'] = v
        for key in ('cbf_status', 'guard_status'):
            total = sum((s.get(key) or {}).values()) or 1
            for k, v in (s.get(key) or {}).items():
                row[f'{key}_{k}'] = v/total
        row['twin_scales'] = json.dumps(s.get('twin_scales'))
        rows.append(row)
    write('s4_closed.csv', rows)


if __name__ == '__main__':
    RESULTS.mkdir(exist_ok=True)
    which = sys.argv[1] if len(sys.argv) > 1 else 'all'
    for name, fn in (('s1', s1), ('s1b', s1b), ('s2', s2), ('s3', s3), ('s4', s4)):
        if which in (name, 'all'):
            fn()
