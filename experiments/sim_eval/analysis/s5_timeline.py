"""Study S5: per-state twin error and true capsule clearance over a whole live-loop run.

    python s5_timeline.py RUN_DIR      -> RUN_DIR/timeline.npz
"""
import json
import sys
from pathlib import Path

import numpy as np

from score_closed import CENTER, HALF, RADIUS, Capsule, default_settings_path, load_settings
from simeval import Run

run = Run(sys.argv[1])
truth = run.truth_at_uv(run.twin['uv'].astype(float))
t, err = [], []
for j, stamp in enumerate(run.twin['times_ns'][::2]):
    k = run.frame(stamp)
    err.append(np.linalg.norm(run.twin['positions'][2*j].astype(float)-truth[k], axis=1).mean()); t.append(stamp/1e9)
cfg = load_settings(str(default_settings_path()))
capsule = Capsule(CENTER, HALF, cfg.top_cap_mode, cfg.top_cap_radius_scale)
ks = np.arange(0, len(run.truth_ns), 6)
clearance = np.array([capsule.distances(run.truth_positions[k].astype(float), RADIUS)[0].min() for k in ks])
releases = [r['material_release'] for r in run.status if 'material_release' in r]
np.savez(run.path/'timeline.npz', t=np.array(t), twin_mean_error=np.array(err), clearance_t=run.truth_ns[ks]/1e9,
         clearance=clearance, releases=json.dumps(releases))
print(run.path.name, 'timeline written', len(t), 'states')
