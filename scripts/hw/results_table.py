#!/usr/bin/env python3
"""Every audited hardware run in one table for the paper (`fabric_run.sh results [--since YYYY-MM-DD]`), 2026-11-21.

One row per run folder (experiments/hardware/sessions/*_fabric_run/<run>) that has an audit.json. Columns come from the
run's own records, never from the current configuration:
  configuration  cell/twin.yaml snapshot: mesh, substeps x iterations, grip model, stretch, self-contact, CBF margin,
                 stretch guard; obstacle: detected (cell/obstacle.yaml) or the virtual box; commit.txt
  timing         twin.status.jsonl: frame time p95 (median over the run), lag median, skipped frames
  motion         review.txt: seconds with commands per robot, largest overshoot past the workspace box
  accuracy       audit.json: twin and estimator vs raw depth (median / MAD / p95 per camera)
  safety         audit.json: real (depth) vs twin clearance to the CBF capsule with commands on; frames with the real
                 sheet inside the capsule or depth inside the obstacle
Writes experiments/hardware/results/runs.csv and runs.md.
"""
import argparse
import csv
import json
import re
import statistics
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / 'experiments/hardware/results'


def status_stats(path):
    frame, lag, skipped = [], [], None
    if not path.exists():
        return {}
    for line in open(path):
        try:
            v = json.loads(line)['values']
        except (ValueError, KeyError):
            continue
        for key, arr in (('frame_ms_p95', frame), ('lag_sec', lag)):
            try:
                arr.append(float(v[key]))
            except (KeyError, TypeError, ValueError):
                pass
        try:
            skipped = float(v['skipped_frames'])
        except (KeyError, TypeError, ValueError):
            pass
    return dict(frame_ms_p95=round(statistics.median(frame), 1) if frame else None,
                lag_ms=round(statistics.median(lag)*1e3) if lag else None, skipped_frames=skipped)


def review_stats(path):
    if not path.exists():
        return {}
    text = path.read_text()
    secs = {r: float(m.group(1)) for r in ('blue', 'silver')
            for m in [re.search(rf'^{r}: commands for ([0-9.]+) s', text, re.M)] if m}
    past = [float(x) for x in re.findall(r'at most ([0-9.]+) cm past the box', text)]
    return dict(commands_s_blue=secs.get('blue'), commands_s_silver=secs.get('silver'),
                box_overshoot_cm=max(past) if past else None)


def config(run):
    out = {}
    try:
        cfg = yaml.safe_load((run / 'cell/twin.yaml').read_text())
        s = cfg.get('sheet', {})
        out.update(mesh=f"{s.get('cells_x')}x{s.get('cells_y')}", solver=f"{s.get('substeps')}x{s.get('iterations')}",
                   grips='point' if s.get('grip_insets_m') else f"patch {s.get('patch_m')} m",
                   stretch=(s.get('stretch') or [None])[0], self_contact=s.get('self_collision'),
                   cbf_margin_m=(cfg.get('cbf') or {}).get('margin_m'),
                   stretch_guard=(cfg.get('stretch_guard') or {}).get('enabled'))
    except (OSError, yaml.YAMLError, AttributeError):
        pass
    try:
        below = float(yaml.safe_load((run / 'cell/twin.yaml').read_text())['obstacle']['center'][2]) < -1.
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        below = False
    out['obstacle'] = 'detected' if (run / 'cell/obstacle.yaml').exists() else ('none' if below else 'virtual box')
    out['commit'] = (run / 'commit.txt').read_text().split()[-1][:7] if (run / 'commit.txt').exists() else None
    return out


def audit_stats(a):
    out = {}
    for name, key in (('twin', 'twin_vs_depth'), ('estimate', 'estimate_vs_depth')):
        for cam, v in sorted((a.get(key) or {}).items()):
            out[f'{name}_depth_median_mm_cam{cam}'] = v.get('median_mm')
            out[f'{name}_depth_mad_mm_cam{cam}'] = v.get('mad_mm')
            out[f'{name}_depth_p95_mm_cam{cam}'] = v.get('p95_abs_mm')
    c = a.get('clearance_commands_on') or {}
    out.update(clear_frames=c.get('frames'), real_min_cm=_cm(c.get('real_min_m')), real_p5_cm=_cm(c.get('real_p5_m')),
               twin_min_cm=_cm(c.get('twin_min_m')), real_minus_twin_median_cm=_cm(c.get('real_minus_twin_median_m')),
               real_minus_twin_p5_cm=_cm(c.get('real_minus_twin_p5_m')), frames_real_inside=c.get('frames_real_below_zero'),
               frames_depth_inside_obstacle=c.get('frames_with_depth_inside_prism'))
    return out


def _cm(x):
    return None if x is None else round(100*x, 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--since', default='', help='only runs from session days >= this date (YYYY-MM-DD)')
    a = ap.parse_args()
    rows = []
    for audit in sorted(REPO.glob('experiments/hardware/sessions/*_fabric_run/*/audit.json')):
        run = audit.parent
        day = run.parent.name[:10]
        if day < a.since:
            continue
        row = dict(day=day, run=run.name, kind=run.name.split('_')[0])
        row.update(config(run))
        row.update(status_stats(run / 'twin.status.jsonl'))
        row.update(review_stats(run / 'review.txt'))
        row.update(audit_stats(json.loads(audit.read_text())))
        rows.append(row)
    OUT.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(OUT / 'runs.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, keys); w.writeheader(); w.writerows(rows)
    short = ['day', 'run', 'mesh', 'solver', 'grips', 'stretch', 'obstacle', 'frame_ms_p95', 'commands_s_blue',
             'twin_depth_median_mm_cam0', 'twin_depth_median_mm_cam1', 'real_min_cm', 'real_p5_cm',
             'real_minus_twin_median_cm', 'frames_real_inside']
    lines = ['| ' + ' | '.join(short) + ' |', '|' + ' --- |'*len(short)]
    lines += ['| ' + ' | '.join('' if r.get(k) is None else str(r.get(k)) for k in short) + ' |' for r in rows]
    (OUT / 'runs.md').write_text('# Hardware runs (generated by scripts/hw/results_table.py)\n\n' + '\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print(f'{len(rows)} runs -> {OUT / "runs.csv"}, {OUT / "runs.md"}')


if __name__ == '__main__':
    main()
