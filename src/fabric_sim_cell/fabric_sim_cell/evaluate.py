"""Score recorded estimator output against the simulated plant's private ground truth.

    ros2 run fabric_sim_cell evaluate PLANT_TRUTH.npz ESTIMATE.npz ESTIMATOR.yaml OUT_DIR [--window NAME START END]

Truth is the plant's cloth (every particle, every cloth frame), interpolated linearly in
simulation time to each estimate's state time and bilinearly in material coordinates to the
estimator's mesh vertices. The estimator's own `metrics.evaluate` then scores it; this
module only aligns the data and adds grip, latency and health figures. Analyses (states at
a capture time) and live predictions are reported separately.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from fabric_sim_cell.core import grid_from_truth, truth_at


def load_mesh(config):
    from fabric_state_estimator.config import load_config
    return load_config(config)[0]


def score(estimate, truth_positions, mesh, label):
    from fabric_state_estimator.metrics import evaluate
    finite = np.isfinite(truth_positions).all(axis=(1, 2))
    if not finite.any():
        return {'label': label, 'count': 0}
    keep = np.flatnonzero(finite)
    est = {key: estimate[key][keep] for key in ('times_ns', 'positions', 'covariance', 'visibility')}
    truth = {'times_ns': est['times_ns'], 'positions': truth_positions[keep]}
    report = evaluate(est, truth)
    corners = mesh.corners
    grip = np.linalg.norm(est['positions'][:, corners] - truth['positions'][:, corners], axis=2)
    per_time = np.linalg.norm(est['positions'] - truth['positions'], axis=2)
    report.update(label=label, states=int(len(keep)),
                  grip_error_m={name: float(np.percentile(grip[:, i], 95)) for i, name in
                                enumerate(('blue', 'silver', 'human_left', 'human_right'))},
                  per_state_p95_m={'median': float(np.median(np.percentile(per_time, 95, axis=1))),
                                   'worst': float(np.percentile(per_time, 95, axis=1).max())},
                  degraded_fraction=float(np.mean(estimate['degraded'][keep])))
    age = (estimate['times_ns'][keep] - estimate['measurement_ns'][keep]) / 1e9
    lag = (estimate['received_ns'][keep] - estimate['measurement_ns'][keep]) / 1e9
    report['state_age_s'] = {'p50': float(np.percentile(age, 50)), 'p95': float(np.percentile(age, 95))}
    report['capture_to_receipt_sim_s'] = {'p50': float(np.percentile(lag, 50)), 'p95': float(np.percentile(lag, 95))}
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('truth'); parser.add_argument('estimate'); parser.add_argument('config'); parser.add_argument('output')
    parser.add_argument('--window', nargs=3, action='append', default=[], metavar=('NAME', 'START_S', 'END_S'),
                        help='Also score states with state time in [START, END) seconds of simulation time')
    options = parser.parse_args(args)
    mesh = load_mesh(options.config)
    with np.load(options.truth, allow_pickle=False) as t:
        truth = {k: t[k] for k in ('times_ns', 'positions', 'material_uv', 'grid_shape')}
    with np.load(options.estimate, allow_pickle=False) as e:
        estimate = {k: e[k] for k in e.files}
    if estimate['positions'].shape[1] != mesh.count:
        raise SystemExit(f'estimate has {estimate["positions"].shape[1]} vertices, config mesh {mesh.count}')
    grid = grid_from_truth(truth['positions'].astype(float), truth['material_uv'], truth['grid_shape'])
    aligned = truth_at(estimate['times_ns'], truth['times_ns'], grid, mesh.uv)
    out = Path(options.output)
    out.mkdir(parents=True, exist_ok=True)
    reports = {}
    groups = {'analysis': ~estimate['predicted'], 'prediction': estimate['predicted']}
    for name, start, end in options.window:
        t = estimate['times_ns'] / 1e9
        window = (t >= float(start)) & (t < float(end))
        groups[f'analysis_{name}'] = window & ~estimate['predicted']
    for name, mask in groups.items():
        if not mask.any():
            reports[name] = {'label': name, 'count': 0}
            continue
        subset = {k: v[mask] for k, v in estimate.items() if isinstance(v, np.ndarray) and len(v) == len(mask)}
        reports[name] = score(subset, aligned[mask], mesh, name)
        if name == 'analysis':
            np.savez(out / 'truth_analysis.npz', times_ns=subset['times_ns'], positions=aligned[mask])
    reports['plant_truth'] = {'frames': int(len(truth['times_ns'])), 'start_s': float(truth['times_ns'][0] / 1e9),
                              'end_s': float(truth['times_ns'][-1] / 1e9)}
    (out / 'metrics.json').write_text(json.dumps(reports, indent=2))
    for name, r in reports.items():
        if 'all' in r and r['all']:
            print(f"{name}: {r['states']} states, p50 {r['all']['p50_m']*1e3:.1f} mm, p95 {r['all']['p95_m']*1e3:.1f} mm, "
                  f"max {r['all']['max_m']*1e3:.1f} mm, 95% coverage {r.get('position_95_percent_coverage', float('nan')):.2f}, "
                  f"degraded {r['degraded_fraction']:.2f}")
    print('SIM_CELL_EVALUATED', out / 'metrics.json')


if __name__ == '__main__':
    main()
