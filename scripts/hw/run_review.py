#!/usr/bin/env python3
"""After a commanded run: did each arm follow, and where did it stop? (`fabric_run.sh review <run_dir>`)

Reads the run's twin.npz (in the run folder, or in /home/taha/fabric_hw_data after `fabric_run.sh save`),
twin.status.jsonl and cell/bridge.yaml. Per arm, in 5 s windows while the twin sent nonzero commands: the
commanded and delivered grip speed, the box faces the grip sits on, how far past the box it went. An arm
that delivers ~0 while commanded is held or stopped: at a face with outward commands it is the box wall;
elsewhere look at the bridge (`fabric_run.sh check`) or the deadman. Also the twin's faults while enabled.
    python3 scripts/hw/run_review.py <run_dir> [--window 5]
"""
import argparse
import collections
import json
from pathlib import Path

import numpy as np
import yaml

DATA = Path('/home/taha/fabric_hw_data')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('run')
    ap.add_argument('--window', type=float, default=5.)
    a = ap.parse_args()
    run = Path(a.run).resolve()
    twin = run / 'twin.npz'
    if not twin.exists():
        twin = DATA / f'{run.parent.name}_{run.name}' / 'twin.npz'
    d = np.load(twin)
    cfg = yaml.safe_load((run / 'cell' / 'bridge.yaml').read_text())
    corners, tc, cmd = d['corners'], d['corners_ns'] / 1e9, d['commands']
    for i, r in enumerate(('blue', 'silver')):
        box = {**cfg['common'], **cfg.get(r, {})}
        lo, hi = np.array(box['workspace_min']), np.array(box['workspace_max'])
        m = cmd[cmd[:, 1] == i]
        tm, v = m[:, 0] / 1e9, m[:, 2:5]
        speed = np.linalg.norm(v, axis=1)
        on = tm[speed > .002]
        if not len(on):
            print(f'{r}: no nonzero commands')
            continue
        t0, t1 = on[0], on[-1]
        sel = (tc >= t0 - 1) & (tc <= t1 + 2)
        p, tt = corners[sel, i], tc[sel]
        beyond = np.linalg.norm(np.maximum(lo - p, 0) + np.maximum(p - hi, 0), axis=1)
        print(f'{r}: commands for {t1 - t0:.0f} s, median {np.median(speed[speed > .002]) * 100:.1f} cm/s; '
              f'grip {np.round(p[0], 3).tolist()} -> {np.round(p[-1], 3).tolist()}; at most {beyond.max() * 100:.1f} cm past the box')
        for s in np.arange(t0, t1, a.window):
            w, wm = (tt >= s) & (tt < s + a.window), (tm >= s) & (tm < s + a.window)
            if w.sum() < 2:
                continue
            moved = np.linalg.norm(p[w][-1] - p[w][0]) / a.window * 100
            end = p[w][-1]
            faces = ''.join(ax + ('-' if end[k] <= lo[k] + .003 else '+') for k, ax in enumerate('xyz')
                            if end[k] <= lo[k] + .003 or end[k] >= hi[k] - .003)
            c = np.mean(v[wm], 0) * 100
            flag = '  <- held/stopped' if np.linalg.norm(c) > 2 and moved < .3 else ''
            print(f'   t+{s - t0:4.0f} s  commanded {np.linalg.norm(c):4.1f} cm/s {np.round(c, 1).tolist()}  delivered {moved:4.1f} cm/s'
                  f'  faces [{faces}]  past {beyond[w].max() * 100:.1f} cm{flag}')
    status = run / 'twin.status.jsonl'
    if status.exists():
        faults, n = collections.Counter(), 0
        for line in status.open():
            values = json.loads(line)['values']
            if values.get('commands_enabled') == 'True':
                n += 1
                faults[values.get('fault') or 'none (commands sent)'] += 1
        if n:
            print('twin while enabled: ' + ', '.join(f'{k} {100 * c / n:.0f} %' for k, c in faults.most_common()))


if __name__ == '__main__':
    main()
