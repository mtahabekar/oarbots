"""Study S4 scoring: one closed-loop carry against the plant's private truth.

    python score_closed.py RUN_DIR [RUN_DIR ...]      -> RUN_DIR/score.json for each

Safety: the true sheet's clearance from the obstacle box, and from the capsule the CBF protects
(the box's enclosing capsule, the same geometry and particle radius the CBF uses), with time and
depth inside the 4 cm margin. Load: the plant's true robot-grip pull (true_tension.py), raw and
0.1 s low-pass. Task: how closely the robots followed the hands (robot-hand distance error against
the formation), commanded speed, and CBF and stretch-guard status counts.
"""
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

from fabric_twin.cbf import default_settings_path
from fabric_twin.vendor import use_capsule_cbf

use_capsule_cbf()
from capsule_cbf.capsule import Capsule  # noqa: E402
from capsule_cbf.settings import load_settings  # noqa: E402

CENTER, HALF = np.array([0., 0., .16]), np.array([.22, .22, .16])
RADIUS, MARGIN = .008, .04


def box_clearance(points, center=CENTER, half=HALF):
    d = np.abs(points-center)-half
    return np.linalg.norm(np.maximum(d, 0.), axis=-1)+np.minimum(d.max(axis=-1), 0.)


def lowpass(t, x, tau=.1):
    out = np.empty_like(x); out[0] = x[0]
    for i in range(1, len(x)):
        a = 1-np.exp(-(t[i]-t[i-1])/tau)
        out[i] = out[i-1]+a*(x[i]-out[i-1])
    return out


def score(run):
    run = Path(run)
    manifest = json.loads((run/'manifest.json').read_text())
    lo, hi = manifest.get('carry_window_s', [12., 54.8])
    cfg = load_settings(str(default_settings_path()))
    capsule = Capsule(CENTER, HALF, cfg.top_cap_mode, cfg.top_cap_radius_scale)
    with np.load(run/'truth.npz') as t:
        times, positions, corners = t['times_ns']/1e9, t['positions'], t['corners']
    window = (times >= lo) & (times <= hi)
    box = np.array([box_clearance(p.astype(float)).min() for p in positions[window]])-RADIUS
    caps = np.array([capsule.distances(p.astype(float), RADIUS)[0].min() for p in positions[window]])
    tw = times[window]
    dt = np.diff(tw, append=tw[-1])
    out = dict(name=run.name, condition=manifest.get('condition'), repeat=manifest.get('repeat'),
               material=manifest.get('material'), twin_scales=manifest.get('twin_scales'),
               box_clearance_min_m=float(box.min()), box_clearance_p5_m=float(np.percentile(box, 5)),
               capsule_clearance_min_m=float(caps.min()),
               margin_time_s=float(dt[caps < MARGIN].sum()), margin_depth_integral_ms=float(((MARGIN-caps).clip(0)*dt).sum()),
               contact_time_s=float(dt[box <= 0.].sum()))
    # how well the robots followed: horizontal robot-hand distance against the starting formation
    k0 = int(np.searchsorted(times, lo))
    start = np.linalg.norm(corners[k0, :2]-corners[k0, 2:], axis=1)
    follow = np.abs(np.linalg.norm(corners[window, :2]-corners[window, 2:], axis=2)-start)
    out['formation_error_p95_m'] = float(np.percentile(follow, 95))
    out['robot_height_mean_m'] = float(corners[window, :2, 2].mean())
    tension = run/'true_tension.npz'
    if tension.exists():
        with np.load(tension) as z:
            tt, pull = z['times'], z['tension']
        m = (tt >= lo) & (tt <= hi)
        robots = pull[m, :2]
        smooth = np.column_stack([lowpass(tt[m], robots[:, i]) for i in range(2)])
        out['pull_robot'] = dict(p50=float(np.median(robots)), p95=float(np.percentile(robots, 95)), max=float(robots.max()),
                                 max_lowpass=float(smooth.max()), p95_lowpass=float(np.percentile(smooth, 95)),
                                 frac_above_6n=float((robots > 6.).mean()))
        out['pull_hands'] = dict(p95=float(np.percentile(pull[m, 2:], 95)), max=float(pull[m, 2:].max()))
    rows = [json.loads(line) for line in (run/'twin.cbf.jsonl').read_text().splitlines() if line.strip()]
    rows = [r for r in rows if lo <= r.get('twin_time', -1) <= hi]
    if rows:
        guard = [r.get('stretch_guard') or {} for r in rows]
        out['cbf_status'] = dict(Counter(r.get('status', '') for r in rows))
        out['guard_status'] = dict(Counter(g.get('status', 'off') for g in guard))
        out['twin_capsule_min_m'] = float(min(r.get('min_clearance', np.inf) for r in rows))
        out['zero_command_fraction'] = float(np.mean([np.allclose(r.get('follower_command', [[0, 0, 0]]), 0.) for r in rows]))
        out['faults'] = dict(Counter(r.get('fault', '') for r in rows if r.get('fault')))
        out['cbf_change_mps_mean'] = float(np.mean([np.linalg.norm(np.subtract(r.get('follower_velocity', [[0]*3]*2),
                                                                             r.get('nominal_velocity', [[0]*3]*2))) for r in rows]))
    if (run/'twin_eval.json').exists():
        out['twin_error_p95_m'] = json.loads((run/'twin_eval.json').read_text())['surface_error_m']['p95']
    (run/'score.json').write_text(json.dumps(out, indent=1))
    return out


if __name__ == '__main__':
    for path in sys.argv[1:]:
        s = score(path)
        print(s['name'], f"box min {s['box_clearance_min_m']*100:.1f} cm, capsule min {s['capsule_clearance_min_m']*100:.1f} cm, "
              f"pull p95 {s.get('pull_robot', {}).get('p95', float('nan')):.1f} N")
