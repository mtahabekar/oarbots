#!/usr/bin/env python3
"""Score named twin configurations on recorded windows (`fabric_run.sh twin-compare --windows ... --configs ...`).

Each configuration is name:mesh:key=value,key=value with keys from fabric_twin.identify.PARAMETERS (log10 scales for
bend/shear/stretch, metres for grip offsets and insets); "@grips=<result.json>" adds a grip_point_search result;
substeps=, iterations=, linear_iterations= change the solver (default 32 x 10, linear 3).
Every configuration free-runs the twin from the recorded grips (point grips) and is
scored on raw depth, as in scripts/hw/warmup_calibrate.py. Writes JSON with every result, per window.

    fabric_run.sh twin-compare --out x.json --windows run:40:120 run2:20:90 \\
        --configs base:14: shear+1:14:shear=1,@grips=g/result.json
"""
import argparse
import json
import multiprocessing as mp
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts/hw'))
import warmup_calibrate as wc  # noqa: E402


def parse(item):
    name, mesh, rest = item.split(':', 2)
    theta, numerics = {}, {}
    for kv in filter(None, rest.split(',')):
        key, value = kv.split('=')
        if key in ('substeps', 'iterations', 'linear_iterations'):
            numerics[key] = int(value)
        elif key == '@grips':
            theta.update({k: float(v) for k, v in json.loads(Path(value).read_text())['calibrated'].items()})
        else:
            theta[key] = float(value)
    return name, int(mesh), theta, numerics


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--windows', nargs='+', required=True, help='run_dir:start:end')
    ap.add_argument('--configs', nargs='+', required=True)
    ap.add_argument('--config', default=str(REPO / 'experiments/hardware/cell/twin.yaml'))
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    configs = [parse(c) for c in a.configs]
    windows = []
    for w in a.windows:
        run, start, end = w.rsplit(':', 2)
        windows.append((str(Path(run).resolve()), f'{start}:{end}'))
    tasks = [dict(config=a.config, cells=m, run=r, window=w, deployed=False, theta=t, numerics=n)
             for _, m, t, n in configs for r, w in windows]
    with mp.get_context('spawn').Pool(wc.safe_workers(a.workers)) as pool:
        out = pool.map(wc._task, tasks, chunksize=1)
    rows = []
    for (name, mesh, theta, _), chunk in zip(configs, [out[i:i+len(windows)] for i in range(0, len(out), len(windows))]):
        for (run, window), r in zip(windows, chunk):
            rows.append(dict(config=name, mesh=mesh, run=Path(run).name, window=window, theta=theta,
                             **{k: v for k, v in r.items() if not k.startswith('frame_')}, frame_ms_mm2=r['frame_ms_mm2'],
                             frame_t=r['frame_t']))
            print(f'{name:>16} {mesh:3d} {Path(run).name:>24} {window:>7}: objective {r["rms_tanh50_mm"]:6.2f} mm, '
                  f'median {r["median_mm"]:+6.1f}, p95 {r["p95_abs_mm"]:6.1f}', flush=True)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(rows))


if __name__ == '__main__':
    main()
