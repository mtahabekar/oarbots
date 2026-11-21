#!/usr/bin/env python3
"""Identify the twin offline from a recorded run (`fabric_run.sh identify <run_dir> ...`).

Runs many parameter hypotheses of the twin sheet as parallel Newton worlds, each free-running from the
measured grips over a window, and scores them on the raw camera depth (fabric_twin/identify.py). Needs the
run's twin.npz and twin.depth.npz (run folder or its /home/taha/fabric_hw_data copy). GPU; no ROS graph,
no robots.

    --mode sensitivity   one batch: which parameters this window can inform (Cramér-Rao sd vs prior sd)
    --mode fit           ES-MDA over --params (default 16 members, 4 iterations)
    --mode evaluate      residual statistics for given values (--set stretch=-1 hand_down=0.04)

Windows: --window warmup (the last warm-up), warmup:0 (the first), or "25:49" (seconds from the first grip
record). Use stretches where the sheet touches nothing (no obstacle in the worlds).
Values are log10 scales for stretch/shear/bend and metres for hand_down/hand_in/robot_down.
--override changes the twin's numerics for a study, e.g. "cells_x=20 cells_y=20 substeps=32 iterations=10".
"""
import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/fabric_twin'))
from fabric_twin.identify import PARAMETERS, Forward, es_mda, make_window, parameter_table, sensitivity, summary  # noqa: E402
from fabric_twin.node import load_twin_config  # noqa: E402


def data_file(run, name):
    for folder in (run, Path('/home/taha/fabric_hw_data') / f'{run.parent.name}_{run.name}'):
        if (folder / name).exists():
            return folder / name
    raise SystemExit(f'{name} not found for {run} (nor in /home/taha/fabric_hw_data)')


def warmup_windows(run):
    found = []
    path = run / 'twin.status.jsonl'
    if not path.exists():
        return found
    for line in open(path):
        if 'warmup_window' in line:
            row = json.loads(line)
            data = row.get('data', row)
            found.append(json.loads(data) if isinstance(data, str) else data)
    return found


def load(run, window, depth_every, points, skip):
    twin, depth = np.load(data_file(run, 'twin.npz')), np.load(data_file(run, 'twin.depth.npz'))
    corners_ns, corners = twin['corners_ns'], twin['corners']
    offsets = np.r_[0, np.cumsum(depth['counts'])]
    points_list = [depth['points'][a:b] for a, b in zip(offsets[:-1], offsets[1:])]
    if window.startswith('warmup'):
        found = warmup_windows(run)
        if not found:
            raise SystemExit('no warm-up window in twin.status.jsonl; give --window START:END seconds')
        w = found[int(window.split(':')[1]) if ':' in window else -1]
        start, end = int(w['start_ns']), int(w['start_ns'] + w['holds'][-1][1] * 1e9)
    else:
        a, b = (float(x) for x in window.split(':'))
        start, end = int(corners_ns[0] + a * 1e9), int(corners_ns[0] + b * 1e9)
    return make_window(corners_ns, corners, depth['stamps_ns'], points_list, start, end, depth_every, points, skip)


def values(text, names):
    theta = np.zeros(len(names))
    for item in (text or []):
        key, value = item.split('=')
        if key not in names:
            raise SystemExit(f'--set {key}: not in --params {names}')
        theta[names.index(key)] = float(value)
    return theta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', type=Path)
    ap.add_argument('--config', type=Path, default=REPO / 'experiments/hardware/cell/twin.yaml')
    ap.add_argument('--window', default='warmup')
    ap.add_argument('--mode', choices=('sensitivity', 'fit', 'evaluate'), default='sensitivity')
    ap.add_argument('--params', default='stretch,shear,bend,hand_down,hand_in')
    ap.add_argument('--set', nargs='*', help='parameter values for evaluate (and the fit/sensitivity centre)')
    ap.add_argument('--override', nargs='*', default=[], help='TwinSpec fields for a numerics study')
    ap.add_argument('--members', type=int, default=16)
    ap.add_argument('--noise-mm', type=float, default=80., help='per-point noise incl. model error (see es_mda)')
    ap.add_argument('--depth-every', type=int, default=2)
    ap.add_argument('--points', type=int, default=64)
    ap.add_argument('--skip-sec', type=float, default=1.)
    ap.add_argument('--json', type=Path, help='write the result here')
    a = ap.parse_args()
    run = a.run.resolve()
    names = [n.strip() for n in a.params.split(',') if n.strip()]
    spec = load_twin_config(a.config)[0]
    if a.override:
        fields = {f.name: f.type for f in dataclasses.fields(spec)}
        changes = {}
        for item in a.override:
            key, value = item.split('=')
            if key not in fields:
                raise SystemExit(f'--override {key}: not a TwinSpec field')
            changes[key] = int(value) if key in ('cells_x', 'cells_y', 'substeps', 'iterations', 'linear_iterations') else float(value)
        spec = dataclasses.replace(spec, **changes)
    window = load(run, a.window, a.depth_every, a.points, a.skip_sec)
    print(f'{run.name}: window {a.window}, {len(window.frame_ns)} frames ({(window.frame_ns[-1] - window.frame_ns[0]) / 1e9:.1f} s), '
          f'{len(window.points)} depth frames, {window.observations} points; mesh {spec.cells_x}x{spec.cells_y}, '
          f'{spec.substeps} substeps x {spec.iterations} iterations; params {names}', flush=True)
    theta0 = values(a.set, names)
    begun = time.perf_counter()
    if a.mode == 'evaluate':
        forward = Forward(spec, window, names)
        result = dict(values=parameter_table(names, theta0)[0], **summary(forward.raw(theta0[None]))[0])
    elif a.mode == 'sensitivity':
        forward = Forward(spec, window, names)
        result = sensitivity(forward, theta0, noise_m=a.noise_mm / 1e3)
    else:
        forward = Forward(spec, window, names)
        result = es_mda(forward, members=a.members, noise_m=a.noise_mm / 1e3, theta0=theta0,
                        progress=lambda h: print(f"  iteration {h['iteration']}: mean {np.round(h['mean'], 3).tolist()} "
                                                 f"sd {np.round(h['sd'], 3).tolist()} cost median {h['cost_mm_median']:.1f} mm "
                                                 f"best {h['cost_mm_best']:.1f} mm", flush=True))
    result.update(run=str(run), window=a.window, mesh=[spec.cells_x, spec.cells_y], substeps=spec.substeps,
                  iterations=spec.iterations, seconds=time.perf_counter() - begun,
                  parameter_units={n: PARAMETERS[n].description for n in names})
    print(json.dumps(result, indent=1, default=float))
    if a.json:
        a.json.parent.mkdir(parents=True, exist_ok=True)
        a.json.write_text(json.dumps(result, indent=1, default=float))


if __name__ == '__main__':
    main()
