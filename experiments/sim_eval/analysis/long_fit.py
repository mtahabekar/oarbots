"""Study S2d: a candidate fix for the bank's start-state bias — output-error identification.

    python long_fit.py RUN_DIR OUT.json [--inputs STATE GRIPS POINTS] [--holds extended]

The production bank scores each candidate on many 2 s free runs, each restarted from the corrected
twin's state. Those states carry the fold pattern the twin formed under its own coefficients, so
the fit inherits the twin's error (S2). Here each candidate is simulated once, from the state at
the start of the warm-up, through the whole excitation with the recorded grips, so it forms its
own folds; it is scored at the end of every hold against the depth captured in that last second,
each capture compared with the candidate's state at the capture's own time (time-resolved). The
coarse-to-fine search is the production one (`Bank.calibrate`), only the cost differs.
"""
import argparse
import json
import time

import numpy as np

from scipy.spatial import cKDTree
from simeval import Run, bank_for, hold_ends, twin_spec


def normal_residuals(q, grid, points, gate_m=.08):
    """Point-to-tangent-plane residuals (m), as in `fabric_twin.bank.surface_cost`, without its
    per-call minimum point count (one camera capture can hold only a few gated points)."""
    surface = q[grid]
    normal = np.cross(np.gradient(surface, axis=1), np.gradient(surface, axis=0))
    normal /= np.maximum(np.linalg.norm(normal, axis=2, keepdims=True), 1e-12)
    flat, normals = surface.reshape(-1, 3), normal.reshape(-1, 3)
    distance, index = cKDTree(flat).query(points)
    keep = distance < gate_m
    return np.einsum('ij,ij->i', points[keep]-flat[index[keep]], normals[index[keep]])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('run'); parser.add_argument('output')
    parser.add_argument('--inputs', nargs=3, default=('twin', 'measured', 'depth'), metavar=('STATE', 'GRIPS', 'POINTS'))
    parser.add_argument('--holds', default='extended')
    parser.add_argument('--points-sec', type=float, default=1.)
    parser.add_argument('--max-points', type=int, default=600, help='per hold')
    options = parser.parse_args()
    state_src, grip_src, point_src = options.inputs
    run = Run(options.run)
    spec, _ = twin_spec(run.path/'twin.yaml')
    bank = bank_for(spec, run)
    cloth = bank.cloth
    start_ns, _ = run.warmup_window()
    ends = hold_ends(run, options.holds)
    k0, k1 = run.frame(start_ns), run.frame(ends[-1])
    frames = np.arange(k0, k1+1); times = run.truth_ns[frames]
    G = run.grips(grip_src, times)
    truth = run.truth_at_uv(cloth.uv)
    if state_src == 'truth':
        q0, v0 = truth[k0].astype(float), run.truth_velocity(truth, k0)
    else:
        j = int(np.argmin(np.abs(run.twin['times_ns']-start_ns)))
        q0, v0 = run.twin['positions'][j].astype(float), run.twin['velocities'][j].astype(float)
    rng = np.random.default_rng(0)
    # observations: per hold, groups of points with their capture times (frame index in the run)
    checks = []
    for end in ends:
        lo = end-int(options.points_sec*1e9)
        if point_src == 'depth':
            d = run.depth
            idx = np.flatnonzero((d['stamps_ns'] >= lo) & (d['stamps_ns'] <= end))
            groups = [(int(np.searchsorted(times, d['stamps_ns'][i])), d['points'][d['offsets'][i]:d['offsets'][i+1]].astype(float)) for i in idx]
        else:
            ks = np.flatnonzero((run.truth_ns >= lo) & (run.truth_ns <= end))
            groups = [(int(k-k0), run.truth_positions[k].astype(float)) for k in ks[::4]]
        total = sum(len(p) for _, p in groups)
        keep = max(1, int(np.ceil(total/options.max_points)))
        groups = [(f, p[::keep]) for f, p in groups if 0 <= f < len(frames)]
        checks.append(groups)
    wanted = {f for groups in checks for f, _ in groups}

    def cost(scales, cases=None):
        cloth.set_material(*scales)
        cloth.set_external(np.zeros((cloth.count, 3)))
        cloth.targets = cloth.to_twin(G[0])
        cloth.set_state(q0, v0)
        states = {}
        for i in range(1, len(frames)):
            cloth.step(cloth.within_reach(G[i]))
            if i in wanted:
                states[i] = cloth.state()[0]
        per_hold = []
        for groups in checks:
            # every capture against the candidate's state at its own time, pooled over the hold
            residuals = [r for f, points in groups for r in normal_residuals(states[f], bank.grid, points)]
            if len(residuals) >= 20:
                per_hold.append(float(np.mean(np.minimum(np.square(residuals), .03**2))))
        if not per_hold:
            return np.inf, [cloth.state()[0]]
        bank.evaluations += 1
        return float(np.mean(per_hold)), [cloth.state()[0]]

    bank.cost = cost
    started = time.perf_counter()
    fit = bank.calibrate([None], (1., 1.))
    true = run.manifest['true_scales']
    out = dict(run=run.path.name, manifest=run.manifest, inputs=dict(state=state_src, grips=grip_src, points=point_src),
               method='output-error, time-resolved', holds=options.holds, fit=fit, fit_seconds=time.perf_counter()-started,
               log_error=dict(bend=float(np.log(fit['bend']/true['bend'])), shear=float(np.log(fit['shear']/true['shear']))))
    json.dump(out, open(options.output, 'w'), indent=1)
    print(f"{run.path.name} long {state_src}/{grip_src}/{point_src}: bend {fit['bend']:.3f} shear {fit['shear']:.3f} "
          f"(true {true['bend']:.3f}, {true['shear']:.3f}) cost {fit['cost_m']*1e3:.2f} mm, {out['fit_seconds']:.0f} s", flush=True)


if __name__ == '__main__':
    main()
