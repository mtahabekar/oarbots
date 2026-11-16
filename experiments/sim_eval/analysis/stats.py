"""Paired statistics for the report (results/stats.json): S3 conditions against B1 per dataset,
pooled over the mismatched datasets and over all; S2 extended vs baseline holds."""
import json
import numpy as np
from figures import RESULTS, paired, read

rows = read('s3_replays.csv')
out = {}
sets = {'mismatched (matched B, C; fine A, B)': lambda r: not (r['structure'] == 'matched' and r['material'] == 'A'),
        'no mismatch (matched A)': lambda r: r['structure'] == 'matched' and r['material'] == 'A',
        'all datasets': lambda r: True}
for name, keep in sets.items():
    sub = [r for r in rows if keep(r)]
    for key in ('validation_mean_mm', 'validation_low_abs_dz_mm', 'forecast_60_mean_mm', 'warmup_mean_mm'):
        for a, b in [('off_nominal', 'est_nominal'), ('off_nominal', 'off_fitted'), ('off_nominal', 'est_fitted'),
                     ('off_nominal', 'off_true'), ('off_fitted', 'est_fitted'), ('est_nominal', 'oracle_nominal'),
                     ('off_nominal', 'off_nominal_truthgrips'), ('est_nominal', 'est_nominal_truthgrips')]:
            out[f'{name} | {key} | {a} -> {b}'] = paired(sub, key, a, b)
fits = [r for r in read('s2_fits.csv') if isinstance(r.get('bend'), float)]
for level in ('L0', 'L3'):
    for coef in ('bend', 'shear'):
        sub = [dict(r, abs_err=abs(r[f'log_err_{coef}'])) for r in fits if r['level'] == level]
        out[f'S2 {level} |log err {coef}| baseline -> extended'] = paired(sub, 'abs_err', 'baseline', 'extended', field='holds')
json.dump(out, open(RESULTS/'stats.json', 'w'), indent=1)
for k, v in out.items():
    if v.get('n') and ('validation_mean' in k or 'S2' in k):
        print(f"{k}: n={v['n']} {v['mean_a']:.2f}->{v['mean_b']:.2f} diff {v['mean_diff']:+.2f} CI [{v['ci'][0]:+.2f},{v['ci'][1]:+.2f}] p={v['p']:.4f} better {v['b_better']}/{v['n']}")
