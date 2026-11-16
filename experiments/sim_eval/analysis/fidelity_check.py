"""Does the offline replay reproduce the live twin? Replays the live configuration (estimator
correction, the twin's own coefficients, the recorded grips) and compares it with the twin states
recorded live, and both with plant truth.

    python fidelity_check.py RUN_DIR OUT.json [--device cpu]
"""
import argparse
import json

import numpy as np

from simeval import Run, replay, twin_spec

parser = argparse.ArgumentParser()
parser.add_argument('run'); parser.add_argument('output'); parser.add_argument('--device', default='cuda:0')
options = parser.parse_args()
run = Run(options.run)
spec, feedback = twin_spec(run.path/'twin.yaml', device=options.device)
result = replay(run, spec, feedback, (1., 1.), 'estimator', 'measured', t0=12., sample_every=1, keep_states_every=4)
offline_t, offline_q, _ = result['states']
live_t, live_q = run.twin['times_ns'], run.twin['positions']
pairs = [(i, j) for i, t in enumerate(offline_t) for j in [int(np.argmin(np.abs(live_t-t)))] if abs(live_t[j]-t) < 5e6]
truth = run.truth_at_uv(run.twin['uv'].astype(float))
gap, off_err, live_err = [], [], []
for i, j in pairs:
    k = run.frame(offline_t[i])
    gap.append(np.linalg.norm(offline_q[i]-live_q[j], axis=1).mean())
    off_err.append(np.linalg.norm(offline_q[i]-truth[k], axis=1).mean())
    live_err.append(np.linalg.norm(live_q[j].astype(float)-truth[k], axis=1).mean())
out = dict(run=run.path.name, pairs=len(pairs), offline_vs_live_mean_mm=1e3*float(np.mean(gap)), offline_vs_live_p95_mm=1e3*float(np.percentile(gap, 95)),
           offline_error_mean_mm=1e3*float(np.mean(off_err)), live_error_mean_mm=1e3*float(np.mean(live_err)),
           times=[float(t)/1e9 for t in offline_t[[i for i, _ in pairs]]], gap_mm=[1e3*g for g in gap],
           offline_error_mm=[1e3*e for e in off_err], live_error_mm=[1e3*e for e in live_err])
json.dump(out, open(options.output, 'w'))
print({k: v for k, v in out.items() if not isinstance(v, list)})
