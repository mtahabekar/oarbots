#!/usr/bin/env python3
"""Compare runs of run_twin_check.sh against the plant's truth: report.md and report.json in OUT_ROOT.

Per run (OUT_ROOT/<mode>_<n>/): from truth.npz, the true sheet's distance to the true table box (every particle,
minus the particle radius) while the robots hold it: minimum, time below 0 (touching or inside) and below the
CBF margin; the true robot-grip speeds; from twin.npz, the commands the twin sent (speed median/p95, time at
the cap); from twin_eval.json, the twin's error against the truth; from plant.log, whether the plant validated.
    python3 twin_check_report.py OUT_ROOT [--start 10]
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
import yaml

PARTICLE_RADIUS = 0.008


def table_distance(points, half):
    """Signed distance (m) of points to the table box standing on the floor, centred on the world origin."""
    local = points-np.array([0., 0., half[2]])
    d = np.abs(local)-half
    return np.linalg.norm(np.maximum(d, 0.), axis=-1)+np.minimum(d.max(axis=-1), 0.)


def run_summary(run, half, start, margin, cap):
    out = dict(run=run.name)
    log = (run / 'plant.log').read_text(errors='replace') if (run / 'plant.log').exists() else ''
    out['plant_validated'] = 'COMBINED_VALIDATED' in log
    if not (run / 'truth.npz').exists():
        out['error'] = 'no truth.npz'
        return out
    t = np.load(run / 'truth.npz')
    times = t['times_ns']/1e9
    sel = times >= start
    q = t['positions'][sel]
    d = table_distance(q, half).min(axis=1)-PARTICLE_RADIUS
    dt = np.diff(times[sel]).mean() if sel.sum() > 1 else 0.
    out.update(true_min_clearance_m=round(float(d.min()), 4), true_time_touching_s=round(float((d < 0).sum()*dt), 2),
               true_time_below_margin_s=round(float((d < margin).sum()*dt), 2), seconds=round(float(sel.sum()*dt), 1))
    corners = t['corners'][sel]
    if len(corners) > 2:
        v = np.linalg.norm(np.diff(corners[:, :2], axis=0), axis=2)/dt          # robot grips: corners 0, 1
        out['true_robot_grip_speed_p95_cmps'] = round(float(np.percentile(v, 95))*100, 1)
        out['true_robot_grip_lowest_z_m'] = [round(float(corners[:, k, 2].min()), 3) for k in (0, 1)]
        out['true_hand_grip_lowest_z_m'] = round(float(corners[:, 2:, 2].min()), 3)
        out['true_sheet_lowest_z_m'] = round(float(q[:, :, 2].min()), 3)
    if (run / 'twin.npz').exists():
        w = np.load(run / 'twin.npz')
        c = w['commands']
        speed = np.linalg.norm(c[:, 2:5], axis=1)
        moving = speed > 1e-4
        if moving.any():
            out.update(twin_command_speed_median_cmps=round(float(np.median(speed[moving]))*100, 1),
                       twin_command_speed_p95_cmps=round(float(np.percentile(speed[moving], 95))*100, 1),
                       twin_command_at_cap_pct=round(float((speed[moving] >= .98*cap).mean())*100, 1))
    if (run / 'twin.cbf.jsonl').exists():
        from collections import Counter
        rows = [json.loads(line) for line in (run / 'twin.cbf.jsonl').open() if line.strip()]
        status = Counter(r.get('status', '') for r in rows if r.get('commands_enabled', True))
        out['twin_cbf_status'] = dict(status)
        clear = [r['min_clearance'] for r in rows if 'min_clearance' in r]
        if clear:
            out['twin_min_capsule_clearance_m'] = round(float(min(clear)), 4)
    if (run / 'twin.status.jsonl').exists():
        vc = sum(json.loads(line)['values'].get('virtual_collision') in ('True', True) for line in (run / 'twin.status.jsonl').open())
        out['twin_virtual_collision_status_msgs'] = vc
    if (run / 'twin_eval.json').exists():
        e = json.loads((run / 'twin_eval.json').read_text())
        out['twin_eval'] = {k: e[k] for k in list(e)[:8]}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('root')
    ap.add_argument('--start', type=float, default=10., help='seconds of simulated time before scoring (the grasp)')
    ap.add_argument('--limits-config', type=Path, help='twin YAML used for the today and limits modes')
    a = ap.parse_args()
    root = Path(a.root)
    sim = Path(os.environ.get('OARBOT_SIM', '/media/taha/ExtermeLinux/oarbot_simulation'))
    half = np.array(json.loads((sim / 'config' / 'fabric_layout.json').read_text())['table_half_extents'], float)
    hardware_margin = .045  # Original Nov. 19 settings for reports without an explicit config.
    if a.limits_config:
        hardware_margin = float(yaml.safe_load(a.limits_config.read_text())['cbf']['margin_m'])
    modes = dict(baseline=dict(margin=.04, cap=.10), today=dict(margin=hardware_margin, cap=.15),
                 limits=dict(margin=hardware_margin, cap=.15),
                 obstacle=dict(margin=.04, cap=.10))
    runs = []
    for run in sorted(p for p in root.iterdir() if p.is_dir()):
        mode = run.name.split('_')[0]
        if mode in modes:
            runs.append(dict(mode=mode, **run_summary(run, half, a.start, **modes[mode])))
    (root / 'report.json').write_text(json.dumps(runs, indent=1))
    cols = ['mode', 'run', 'plant_validated', 'true_min_clearance_m', 'true_time_touching_s', 'true_time_below_margin_s',
            'true_sheet_lowest_z_m', 'true_hand_grip_lowest_z_m', 'true_robot_grip_lowest_z_m', 'true_robot_grip_speed_p95_cmps',
            'twin_min_capsule_clearance_m', 'twin_cbf_status', 'twin_virtual_collision_status_msgs',
            'twin_command_speed_median_cmps', 'twin_command_speed_p95_cmps', 'twin_command_at_cap_pct']
    lines = ['| ' + ' | '.join(cols) + ' |', '|' + ' --- |'*len(cols)]
    lines += ['| ' + ' | '.join(str(r.get(c, '')) for c in cols) + ' |' for r in runs]
    (root / 'report.md').write_text(f'# Twin check, {root.name}\n\nTrue sheet vs true table from {a.start:.0f} s simulated '
                                    '(clearance: every particle, minus its radius).\n\n' + '\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
