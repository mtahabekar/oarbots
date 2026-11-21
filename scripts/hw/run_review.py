#!/usr/bin/env python3
"""After a commanded run: did each arm follow, and where did it stop? (`fabric_run.sh review <run_dir>`)

Reads the run's twin.npz (in the run folder, or in /home/taha/fabric_hw_data after `fabric_run.sh save`),
twin.status.jsonl and cell/bridge.yaml. Per arm, in 5 s windows while the twin sent nonzero commands: the
commanded and delivered grip speed, the box faces the grip sits on, how far past the box it went. An arm
that delivers ~0 while commanded is held or stopped: at a face with outward commands it is the box wall;
elsewhere look at the bridge (`fabric_run.sh check`) or the deadman. Also the twin's faults while enabled, and
for each warm-up: commands on or off, the grip inset each robot actually reached at each hold against the
script's target, and the parameter bank's fit (residual before/after, released or not).
    python3 scripts/hw/run_review.py <run_dir> [--window 5]
"""
import argparse
import collections
import json
import time
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
    bridge = run / 'cell' / 'bridge.yaml'
    cfg = yaml.safe_load(bridge.read_text()) if bridge.exists() else None  # shadow and warm-up runs have none
    corners, tc, cmd = d['corners'], d['corners_ns'] / 1e9, d['commands']
    for i, r in enumerate(('blue', 'silver')):
        if cfg:
            box = {**cfg['common'], **cfg.get(r, {})}
            lo, hi = np.array(box['workspace_min']), np.array(box['workspace_max'])
        else:
            lo, hi = np.full(3, -np.inf), np.full(3, np.inf)
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
        span = ', '.join(f'{ax} {(p[:, k].min() - p[0, k]) * 100:+.0f}/{(p[:, k].max() - p[0, k]) * 100:+.0f} cm' for k, ax in enumerate('xyz'))
        print(f'{r}: commands for {t1 - t0:.0f} s, grip range from its start: {span}')
        print(f'   median {np.median(speed[speed > .002]) * 100:.1f} cm/s; '
              f'grip {np.round(p[0], 3).tolist()} -> {np.round(p[-1], 3).tolist()}; at most {beyond.max() * 100:.1f} cm past the box')
        for s in np.arange(t0, t1, a.window):
            w, wm = (tt >= s) & (tt < s + a.window), (tm >= s) & (tm < s + a.window)
            if w.sum() < 2:
                continue
            # path speed, sampled at 5 Hz (net displacement under-reads motion back and forth)
            pw, tw = p[w], tt[w]
            keep = np.searchsorted(tw, np.arange(tw[0], tw[-1] + 1e-9, .2))
            pw = pw[np.minimum(keep, len(pw) - 1)]
            moved = np.sum(np.linalg.norm(np.diff(pw, axis=0), axis=1)) / max(tw[-1] - tw[0], 1e-6) * 100
            end = p[w][-1]
            faces = ''.join(ax + ('-' if end[k] <= lo[k] + .003 else '+') for k, ax in enumerate('xyz')
                            if end[k] <= lo[k] + .003 or end[k] >= hi[k] - .003)
            c = np.mean(v[wm], 0) * 100
            cs = np.mean(np.linalg.norm(v[wm], axis=1)) * 100 if wm.any() else 0.
            flag = '  <- held/stopped' if cs > 2 and moved < .3 else ''
            print(f'   t+{s - t0:4.0f} s  commanded {cs:4.1f} cm/s (mean vector {np.round(c, 1).tolist()})  delivered {moved:4.1f} cm/s'
                  f'  faces [{faces}]  past {beyond[w].max() * 100:.1f} cm{flag}')
    status = run / 'twin.status.jsonl'
    if status.exists():
        faults, n = collections.Counter(), 0
        rows = [json.loads(line) for line in status.open()]
        for row in rows:
            values = row.get('values', {})  # warm-up windows and bank status are rows without values
            if values.get('commands_enabled') == 'True':
                n += 1
                faults[values.get('fault') or 'none (commands sent)'] += 1
        if n:
            print('twin while enabled: ' + ', '.join(f'{k} {100 * c / n:.0f} %' for k, c in faults.most_common()))
        warmups(rows, corners, tc)


def warmups(rows, corners, tc):
    """Each warm-up the twin announced: commands on?, insets reached per hold, and the bank's result."""
    status = [(r['stamp_ns'] / 1e9, r['values']) for r in rows if 'values' in r]
    windows = [r['data'] for r in rows if r.get('topic') == '/fabric/twin/warmup_window']
    bank = [r['data'] for r in rows if r.get('topic') == '/fabric/bank/status']
    releases = [r['material_release'] for r in rows if 'material_release' in r]
    starts = []
    for w in windows:
        t0 = w['start_ns'] / 1e9
        end = t0 + max(h[1] for h in w['holds'])
        inside = [v for t, v in status if t0 <= t <= end]
        if inside:
            m = json.loads(inside[0].get('material') or '{}')
            starts.append((m.get('bend', 1.), m.get('shear', 1.)))
            tag = ' (NOMINAL: a fit that keeps it proves nothing)' if (m.get('version', 0) == 0 and starts[-1] == (1., 1.)) else ''
            print(f'warm-up start coefficients: bend x{starts[-1][0]:g}, shear x{starts[-1][1]:g} '
                  f'(material version {m.get("version", 0)}){tag}')
        on = sum(v.get('commands_enabled') == 'True' for v in inside)
        stale = sum(bool(v.get('fault')) for v in inside)
        print(f'warm-up at {time.strftime("%H:%M:%S", time.localtime(t0))}: commands on {100 * on / max(len(inside), 1):.0f} % '
              f'of the window, a twin fault {100 * stale / max(len(inside), 1):.0f} %')
        k0 = np.searchsorted(tc, t0)
        if k0 >= len(tc):
            print('   no grip recording in the window')
            continue
        start = corners[k0]
        direction = start[2:] - start[:2]
        direction[:, 2] = 0.
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        plan = {round(h['start'], 2): h for h in w.get('plan', [])}   # since 2026-11-21: label and 3D target per hold
        for a, b, target in w['holds']:
            sel = (tc >= t0 + a) & (tc <= t0 + b)
            if not sel.any():
                continue
            reached = np.einsum('kij,ij->ki', corners[sel, :2] - start[:2], direction).mean(0)
            hold = plan.get(round(a, 2))
            extra = ''
            if hold:   # the last 2 s of the hold, against the full 3D target (lifts, shifts, narrowing)
                late = (tc >= t0 + max(a, b - 2.)) & (tc <= t0 + b)
                error = np.linalg.norm(np.median(corners[late, :2] - start[:2], axis=0) - np.array(hold['target']), axis=1)
                extra = f'  {hold["label"]:<19} 3D error ' + ', '.join(f'{r} {e * 1000:3.0f} mm' for r, e in zip(('blue', 'silver'), error))
            print(f'   hold t+{a:4.1f}-{b:4.1f} s  target inset {target * 100:3.0f} cm  reached '
                  + ', '.join(f'{r} {x * 100:4.1f} cm' for r, x in zip(('blue', 'silver'), reached)) + extra)
        moved = np.abs(np.einsum('kij,ij->ki', corners[(tc >= t0) & (tc <= end), :2] - start[:2], direction)).max()
        tail = (tc >= end - .5) & (tc <= end)
        if tail.any():
            delta = np.median(corners[tail, :2], axis=0) - start[:2]
            print('   full 3D return error: ' + ', '.join(
                f'{robot} {np.linalg.norm(error)*1000:.1f} mm (xyz {np.round(error*1000, 1).tolist()} mm)'
                for robot, error in zip(('blue', 'silver'), delta)))
        smallest = min(h[2] for h in w['holds'] if h[2] > 0)
        if moved < smallest / 2:
            print(f'   ROBOTS DID NOT MOVE (at most {moved * 1000:.0f} mm toward the hands): any fit from this window is meaningless')
    for b in bank:
        if b.get('message') in ('calibrated', 'calibration stopped'):
            r = b['result']
            if 'stop_reason' in r:
                print(f'bank stop: {r["stop_reason"]}, converged {r["converged"]}, '
                      f'{r["rounds_completed"]} rounds, {r["evaluations"]} evaluations')
            gain = 100 * (1 - r['cost_m'] / r['start_cost_m']) if r['start_cost_m'] else 0.
            print(f'bank fit: bend x{r["bend"]:.3f}, shear x{r["shear"]:.3f}; residual {r["start_cost_m"] * 1e3:.1f} -> '
                  f'{r["cost_m"] * 1e3:.1f} mm ({gain:.1f} % lower), {r["cases"]} holds, {r["seconds"]:.0f} s, informative {r["informative"]}')
            if starts:
                s0 = r.get('start') or starts[min(len(starts), [x.get('message') for x in bank].count('calibrated')) - 1]
                reach = r.get('reach', 2 ** 2.75)  # older banks: span 4, 3 rounds
                edge = [n for n, f, a in zip(('bend', 'shear'), (r['bend'], r['shear']), s0)
                        if abs(np.log(f / a)) >= np.log(reach) - 1e-3 or not 1 / 16 * 1.001 < f < 16 * .999]
                print(f'   moved from the start (bend x{s0[0]:g}, shear x{s0[1]:g}): bend x{r["bend"] / s0[0]:.2f}, '
                      f'shear x{r["shear"] / s0[1]:.2f}'
                      + (f'; {" and ".join(edge)} AT THE SEARCH EDGE (x{reach:.3g} from the start): no minimum found' if edge else ''))
        elif b.get('message', '').startswith('calibration skipped'):
            print(f'bank: {b["message"]}')
    if windows and not any(b.get('message') in ('calibrated', 'calibration stopped') for b in bank):
        print('bank: no completed fit in this recording (the run ended before the fit, about 4 min)')
    for rel in releases:
        print(f'material release {rel["version"]} ({rel["source"]}): bend x{rel["bend"]:.3f}, shear x{rel["shear"]:.3f}')


if __name__ == '__main__':
    main()
