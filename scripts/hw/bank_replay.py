#!/usr/bin/env python3
"""Repeat a recorded warm-up's parameter-bank fit offline (`fabric_run.sh bank-replay <run_dir> ...`).

Needs the run's twin.npz, twin.depth.npz (warm-up runs record it since 2026-11-20) and estimate.npz,
in the run folder or its fabric_hw_data copy, and the warm-up windows in twin.status.jsonl. Builds
each case as the bank node does (bank_node.case: twin state `relax` s before the hold's end, the
grips since, the depth of the last second), then either:

    --grid "0.0625 0.25 1 4 16"      cost of every bend x shear pair in that list, with the share of
                                     depth points past the residual clip (a saturated cost cannot rank)
    --start "10 0.1"                 the node's search (span 8, 4 rounds, convergence tolerances)
    --start "10 0.1" --workers 2 --compare --max-evaluations 6 --json comparison.json
                                     sequential/parallel parity and wall time on identical cases;
                                     comparison disables the time cap so both test the same candidates

Scales are of --config's bend and shear (default: the cell's twin.yaml now). GPU; no ROS, no robots.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/fabric_twin'))
from fabric_twin.bank import Bank, Case  # noqa: E402
from fabric_twin.node import load_twin_config  # noqa: E402

RELAX_SEC, POINTS_SEC, MAX_POINTS = 2., 1., 2500   # bank_node defaults


def data_file(run, name):
    for folder in (run, Path('/home/taha/fabric_hw_data') / f'{run.parent.name}_{run.name}'):
        if (folder / name).exists():
            return folder / name
    raise SystemExit(f'{name} not found for {run} (nor in /home/taha/fabric_hw_data)')


def windows(run):
    found = []
    for line in open(run / 'twin.status.jsonl'):
        if 'warmup_window' in line:
            row = json.loads(line)
            data = row.get('data', row)
            found.append(json.loads(data) if isinstance(data, str) else data)
    return found


class Recording:
    def __init__(self, run):
        twin, depth, estimate = (np.load(data_file(run, n)) for n in ('twin.npz', 'twin.depth.npz', 'estimate.npz'))
        self.state_ns, self.q, self.v = twin['times_ns'], twin['positions'].astype(float), twin['velocities'].astype(float)
        self.corner_ns, self.corners = twin['corners_ns'], twin['corners'].astype(float)
        offsets = np.r_[0, np.cumsum(depth['counts'])]
        points = depth['points']  # decompress the NPZ member once, rather than once per camera frame
        self.depth_ns, self.depth_points = depth['stamps_ns'], [points[a:b].astype(float) for a, b in zip(offsets[:-1], offsets[1:])]
        usable = ~estimate['predicted']
        self.estimate_ns, self.estimate = estimate['measurement_ns'][usable], estimate['positions'][usable]
        self.random = np.random.default_rng(0)

    def case(self, end_ns, bank, measured):
        start_ns = end_ns - int(RELAX_SEC * 1e9)
        k = int(np.argmin(abs(self.state_ns - start_ns)))
        if abs(self.state_ns[k] - start_ns) > .2e9:
            return None, 'no twin state'
        grips = self.corners[(self.corner_ns > self.state_ns[k]) & (self.corner_ns <= end_ns)]
        chosen = [p for t, p in zip(self.depth_ns, self.depth_points) if end_ns - int(POINTS_SEC * 1e9) <= t <= end_ns]
        if len(grips) < 30 or not chosen:
            return None, 'no grips or depth'
        points = np.concatenate(chosen)
        if len(points) > MAX_POINTS:
            points = points[self.random.choice(len(points), MAX_POINTS, replace=False)]
        return Case(self.q[k], grips, points, self.v[k]), ''


def clipped(bank, ends, cases, gate=.08, clip=.03):
    """Share of gated depth points whose residual is past the clip, over all cases."""
    from scipy.spatial import cKDTree
    over = total = 0
    for q, case in zip(ends, cases):
        surface = q[bank.grid]
        normal = np.cross(np.gradient(surface, axis=1), np.gradient(surface, axis=0))
        normal /= np.maximum(np.linalg.norm(normal, axis=2, keepdims=True), 1e-12)
        flat, normals = surface.reshape(-1, 3), normal.reshape(-1, 3)
        distance, index = cKDTree(flat).query(case.points)
        keep = distance < gate
        residual = np.einsum('ij,ij->i', case.points[keep] - flat[index[keep]], normals[index[keep]])
        over += int((abs(residual) > clip).sum()); total += int(keep.sum())
    return over / max(total, 1), total


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', type=Path)
    ap.add_argument('--config', type=Path, default=REPO / 'experiments/hardware/cell/twin.yaml')
    ap.add_argument('--window', type=int, default=-1, help='which warm-up window (default: the last)')
    ap.add_argument('--case-start', choices=('twin',), default='twin')
    ap.add_argument('--grid', help='scales to try on both axes, e.g. "0.0625 0.25 1 4 16"')
    ap.add_argument('--start', help='run the search from "bend shear"')
    ap.add_argument('--workers', type=int, default=1, help='independent GPU search workers (requires CUDA MPS)')
    ap.add_argument('--compare', action='store_true', help='compare sequential and parallel fits on identical cases')
    ap.add_argument('--max-evaluations', type=int, default=80)
    ap.add_argument('--max-seconds', type=float, default=600.)
    ap.add_argument('--compare-tolerance-mm', type=float, default=.1)
    ap.add_argument('--json', type=Path, help='write fit or comparison, including candidate tables and timings')
    a = ap.parse_args()
    if a.workers < 1 or (a.compare and (not a.start or a.workers < 2)):
        ap.error('--compare requires --start and --workers >= 2; workers must be positive')
    if a.compare_tolerance_mm < 0 or not np.isfinite(a.compare_tolerance_mm):
        ap.error('--compare-tolerance-mm must be finite and nonnegative')
    run = a.run.resolve()
    found = windows(run)
    if not found:
        raise SystemExit('no warm-up window in twin.status.jsonl')
    window = found[a.window]
    rec = Recording(run)
    spec = load_twin_config(a.config)[0]
    first = int(np.searchsorted(rec.corner_ns, window['start_ns']))
    bank = Bank(spec, rec.corners[min(first, len(rec.corners) - 1)])
    cases = []
    for _, end, inset in window['holds']:
        case, why = rec.case(int(window['start_ns'] + end * 1e9), bank, a.case_start == 'measured')
        print(f'hold ending t+{end:4.1f} s (inset {inset * 100:.0f} cm): ' + (f'{len(case.points)} depth points' if case else why))
        if case:
            cases.append(case)
    if len(cases) < 2:
        raise SystemExit('fewer than two usable holds')
    print(f'{run.name}: window {a.window} of {len(found)}, case start {a.case_start}, config {a.config}')
    if a.grid:
        values = [float(x) for x in a.grid.split()]
        rows = []
        for b in values:
            for s in values:
                t0 = time.perf_counter()
                cost, ends = bank.cost((b, s), cases)
                share, total = clipped(bank, ends, cases)
                rows.append((b, s, np.sqrt(cost)))
                print(f'  bend x{b:<7g} shear x{s:<7g} cost {np.sqrt(cost) * 1e3:6.2f} mm  past the clip {share * 100:4.0f} % '
                      f'of {total} points  ({time.perf_counter() - t0:.1f} s)', flush=True)
        costs = np.array([r[2] for r in rows])
        best = rows[int(np.argmin(costs))]
        print(f'best bend x{best[0]:g}, shear x{best[1]:g} at {best[2] * 1e3:.2f} mm; '
              f'spread over the grid {(costs.max() - costs.min()) * 1e3:.2f} mm')
    if a.start:
        start = tuple(float(x) for x in a.start.split())
        fits = []
        for workers in ([1, a.workers] if a.compare else [a.workers]):
            print(f'search: {workers} requested worker(s)', flush=True)
            begun = time.perf_counter()
            result = bank.calibrate(cases, start, span=8., rounds=4, cost_tolerance_m=.0001, parameter_tolerance=.1,
                                    max_seconds=None if a.compare else a.max_seconds, max_evaluations=a.max_evaluations,
                                    workers=workers,
                                    progress=lambda p: print(f'  {p["evaluations"]:3d} evaluations: best bend x{p["bend"]:.3g}, '
                                                             f'shear x{p["shear"]:.3g}, {p["cost_m"] * 1e3:.2f} mm', flush=True))
            result['seconds'] = time.perf_counter()-begun  # includes worker startup and cleanup
            fits.append(result)
        table = result['table']
        spread = max(r['cost_m'] for r in table) - result['cost_m']
        print(json.dumps({k: v for k, v in result.items() if k not in ('history', 'table')}, default=float))
        print(f'from x{start[0]:g}, x{start[1]:g}: bend x{result["bend"]:.3g}, shear x{result["shear"]:.3g}; '
              f'{result["stop_reason"]}; residual {result["start_cost_m"] * 1e3:.2f} -> {result["cost_m"] * 1e3:.2f} mm; '
              f'cost spread {spread * 1e3:.2f} mm')
        output = dict(run=str(run), config=str(a.config), cases=len(cases), fits=fits)
        passed = True
        if a.compare:
            serial, parallel = fits
            tolerance = a.compare_tolerance_mm / 1000
            same_trials = ([(r['bend'], r['shear']) for r in serial['table']] ==
                           [(r['bend'], r['shear']) for r in parallel['table']])
            differences = [abs(x['cost_m']-y['cost_m']) for x, y in zip(serial['table'], parallel['table'])]
            same_result = all(serial[k] == parallel[k] for k in
                              ('bend', 'shear', 'evaluations', 'converged', 'stop_reason', 'rounds_completed'))
            passed = bool(parallel['workers'] > 1 and same_trials and same_result and
                          np.isfinite(differences).all() and max(differences, default=np.inf) <= tolerance and
                          abs(serial['sensitivity_m']-parallel['sensitivity_m']) <= tolerance)
            output['comparison'] = dict(passed=passed, same_trials=same_trials, same_result=same_result,
                                        tolerance_mm=a.compare_tolerance_mm,
                                        max_cost_difference_mm=max(differences, default=np.inf)*1000,
                                        speedup=serial['seconds']/parallel['seconds'])
            print(json.dumps(output['comparison']), flush=True)
        if a.json:
            a.json.parent.mkdir(parents=True, exist_ok=True)
            a.json.write_text(json.dumps(output, indent=2)+'\n')
        if not passed:
            raise SystemExit('Sequential/parallel comparison FAILED (or parallel workers unavailable)')


if __name__ == '__main__':
    main()
