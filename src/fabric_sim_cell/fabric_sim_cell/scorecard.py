"""Score simulated-cell runs against the project's proposed acceptance targets.

    ros2 run fabric_sim_cell scorecard RESULTS_DIR [--window attached]

RESULTS_DIR holds experiment folders (`<name>/eval/metrics.json`), directly or inside repeat
folders (`set_a/<name>/...`); runs of the same experiment name are pooled. Targets are the online
(Tier B) proposals in docs/fabric-state-estimation-research.md, section 15. Writes
`scorecard.md` and `scorecard.json` into RESULTS_DIR and prints the table.
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

SURFACE_P95_M = 0.050          # online surface accuracy, whole sheet
TIER_A_VISIBLE_P95_M = 0.015   # offline reference target (upper end of 10-15 mm), shown for context
HAND_P95_M = (0.020, 0.030)    # proposed hand position p95
AGE_P95_S = 0.100              # capture-to-state age at <= 0.1 m/s
COVERAGE = 0.95                # nominal 95 % interval coverage


def runs_by_experiment(root):
    found = defaultdict(list)
    for metrics in sorted(Path(root).glob('**/eval/metrics.json')):
        run = metrics.parent.parent
        found[run.name].append(run)
    return found


def summary(values, worst):
    values = [v for v in values if v is not None and np.isfinite(v)]
    if not values:
        return None
    return {'median': float(np.median(values)), 'worst': float(worst(values)), 'runs': len(values)}


def fmt(s, scale=1.0, unit='', digits=1):
    if s is None:
        return 'not recorded'
    text = f"{s['median'] * scale:.{digits}f}{unit}"
    if s['runs'] > 1:
        text += f" (worst {s['worst'] * scale:.{digits}f}{unit}, {s['runs']} runs)"
    return text


def verdict(s, limit, larger_is_worse=True):
    if s is None:
        return 'n/a'
    worst_ok = s['worst'] <= limit if larger_is_worse else s['worst'] >= limit
    median_ok = s['median'] <= limit if larger_is_worse else s['median'] >= limit
    return 'met' if worst_ok else ('met in the median, not every run' if median_ok else 'not met')


def experiment_row(runs, window):
    m = [json.loads((r / 'eval' / 'metrics.json').read_text()).get(f'analysis_{window}', {}) for r in runs]
    m = [x for x in m if x.get('states')]
    if not m:
        return None
    hands = [max((h.get('p95_m', np.nan) for h in x.get('hand_error_m', {}).values() if h.get('count')), default=None)
             for x in m]
    hand_counts = [sum(h.get('count', 0) for h in x.get('hand_error_m', {}).values()) for x in m]
    chains = []
    for r in runs:
        f = r / 'ft_chain.json'
        if f.exists():
            chain = json.loads(f.read_text())
            q = [c.get('quasi_static_error_N', {}) or {} for c in chain.values() if isinstance(c, dict)]
            chains.append(max((x.get('p95', np.nan) for x in q), default=None))
    return {
        'runs': len(m),
        'surface_p95_m': summary([x['all']['p95_m'] for x in m], max),
        'surface_p50_m': summary([x['all']['p50_m'] for x in m], max),
        'observed_p95_m': summary([(x.get('observed') or {}).get('p95_m') for x in m], max),
        'inferred_p95_m': summary([(x.get('inferred') or {}).get('p95_m') for x in m], max),
        'stale_p95_m': summary([(x.get('stale') or {}).get('p95_m') for x in m], max),
        'hand_p95_m': summary(hands, max),
        'hand_states': int(np.median(hand_counts)) if any(hand_counts) else 0,
        'age_p95_s': summary([x['capture_to_receipt_sim_s']['p95'] for x in m], max),
        'coverage': summary([x['position_95_percent_coverage'] for x in m], min),
        'analyses': summary([x['states'] for x in m], min),
        'ft_quasi_static_p95_n': summary(chains, max),
    }


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('root')
    parser.add_argument('--window', default='attached', help='Scored window name given to evaluate (default attached)')
    options = parser.parse_args(args)
    root = Path(options.root)
    table = {name: experiment_row(runs, options.window) for name, runs in runs_by_experiment(root).items()}
    table = {k: v for k, v in table.items() if v}
    if not table:
        raise SystemExit(f'no scored runs with window {options.window!r} under {root}')
    lines = [f'# Simulated-cell scorecard ({root})', '',
             f'Window `{options.window}`; median over runs, worst run in brackets. Targets: online (Tier B) '
             'proposals, docs/fabric-state-estimation-research.md section 15.', '',
             '| Experiment | Requirement | Target | Measured | Verdict |', '| --- | --- | --- | --- | --- |']
    for name, r in table.items():
        rows = [
            ('Surface accuracy, whole sheet', 'p95 <= 50 mm', fmt(r['surface_p95_m'], 1e3, ' mm'),
             verdict(r['surface_p95_m'], SURFACE_P95_M)),
            ('Surface accuracy, observed vertices', 'context: Tier A reference p95 10-15 mm',
             fmt(r['observed_p95_m'], 1e3, ' mm'), verdict(r['observed_p95_m'], TIER_A_VISIBLE_P95_M) + ' (reference)'),
            ('Hand position', 'p95 <= 20-30 mm (proposed)',
             fmt(r['hand_p95_m'], 1e3, ' mm') + (f", {r['hand_states']} hand samples" if r['hand_states'] else ''),
             verdict(r['hand_p95_m'], HAND_P95_M[1]) if r['hand_p95_m'] else 'n/a'),
            ('Uncertainty calibration', '95 % interval covers ~95 %', fmt(r['coverage'], 1, '', 2),
             'n/a' if r['coverage'] is None else ('consistent or conservative' if r['coverage']['worst'] >= 0.90
                                                  else 'overconfident')),
            ('Capture-to-state age', 'p95 <= 100 ms at <= 0.1 m/s', fmt(r['age_p95_s'], 1e3, ' ms', 0),
             'simulation-time age; wall-clock latency not recorded'),
        ]
        if r['ft_quasi_static_p95_n'] is not None:
            rows.append(('F/T compensation residual (slow motion)', 'small vs cloth load (5-8 N)',
                         fmt(r['ft_quasi_static_p95_n'], 1, ' N', 2), 'measured'))
        for i, (req, target, measured, v) in enumerate(rows):
            label = '{} ({} run{})'.format(name, r['runs'], '' if r['runs'] == 1 else 's') if i == 0 else ''
            lines.append(f'| {label} | {req} | {target} | {measured} | {v} |')
    text = '\n'.join(lines) + '\n'
    (root / 'scorecard.md').write_text(text)
    (root / 'scorecard.json').write_text(json.dumps(table, indent=1))
    print(text)
    print('SCORECARD_WRITTEN', root / 'scorecard.md')


if __name__ == '__main__':
    main()
