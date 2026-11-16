"""Figures and paired statistics for the sim_eval report (reads results/*.csv, writes figures/).

    python figures.py [s1 s2 s3 s4 ...]
"""
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.stats import wilcoxon

import plotstyle as ps
from plotstyle import plt

HERE = Path(__file__).resolve().parent
RESULTS, FIGURES = HERE.parent/'results', HERE.parent/'figures'
GROUPS = [('matched', 'A'), ('matched', 'B'), ('matched', 'C'), ('fine', 'A'), ('fine', 'B')]


def read(name):
    path = RESULTS/name
    if not path.exists():
        return []
    rows = list(csv.DictReader(open(path)))
    for r in rows:
        for k, v in r.items():
            try:
                r[k] = float(v)
            except (TypeError, ValueError):
                pass
    return rows


def bootstrap_ci(values, n=10000, seed=0):
    values = np.asarray(values, float)
    if len(values) < 2:
        return np.nan, np.nan
    rng = np.random.default_rng(seed)
    means = values[rng.integers(0, len(values), (n, len(values)))].mean(axis=1)
    return float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def paired(rows, key, a, b, field='condition', by='dataset'):
    """Paired differences b - a of `key` across datasets, with bootstrap CI and Wilcoxon p."""
    va = {r[by]: r[key] for r in rows if r[field] == a}
    vb = {r[by]: r[key] for r in rows if r[field] == b}
    common = sorted(set(va) & set(vb))
    d = np.array([vb[k]-va[k] for k in common])
    if len(d) == 0:
        return dict(n=0)
    lo, hi = bootstrap_ci(d)
    p = float(wilcoxon(d).pvalue) if len(d) >= 5 and np.any(d != 0) else np.nan
    return dict(n=len(d), mean_a=float(np.mean([va[k] for k in common])), mean_b=float(np.mean([vb[k] for k in common])),
                mean_diff=float(d.mean()), ci=(lo, hi), p=p, b_better=int((d < 0).sum()))


def group_label(structure, material):
    return f'{structure} {material}'


# ------------------------------------------------------------------ S1
def fig_s1():
    hands, est = read('s1_hands.csv'), read('s1_estimator.csv')
    if not hands:
        return
    fig, axes = plt.subplots(1, 3, figsize=(ps.DOUBLE_COLUMN, 2.2))
    specs = [(hands, 'hand_error_p95_mm', 'Glove grip error p95 (mm)'),
             (hands, 'hand_unseen_frac', 'Time a hand is unseen > 0.2 s'),
             (est, 'est_error_p95_mm', 'Estimator interior error p95 (mm)')]
    for ax, (rows, key, title) in zip(axes, specs):
        for g, (structure, material) in enumerate(GROUPS):
            for p, phase in enumerate(('warmup', 'validation')):
                vals = [r[key] for r in rows if r['structure'] == structure and r['material'] == material and r['phase'] == phase]
                x = g+(p-.5)*.3
                ax.scatter(np.full(len(vals), x), vals, s=16, color=ps.color(p), marker=ps.MARKERS[p], edgecolor=ps.SURFACE,
                           linewidth=.8, zorder=3, label=('warm-up (hands still)' if phase == 'warmup' else 'held-out motion (hands move)')
                           if g == 0 else None)
        ax.set_xticks(range(len(GROUPS)), [group_label(*g) for g in GROUPS], rotation=30, ha='right')
        ax.set_xlim(-.6, len(GROUPS)-.4); ax.set_ylim(bottom=0)
        ax.set_title(title, fontsize=8)
        if key == 'hand_unseen_frac':
            ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{v:.0%}'))
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=2, bbox_to_anchor=(.5, -.08))
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s1_perception')


def fig_s1b():
    rows = read('s1b_observer.csv')
    if not rows:
        return
    conds = [('novision', 'grips only\n(no camera factors)'), ('vision', 'cameras +\nglove grips'), ('oracle_vision', 'cameras +\ntrue hand grips')]
    fig, axes = plt.subplots(1, 2, figsize=(ps.DOUBLE_COLUMN, 2.4), sharey=True)
    for ax, phase in zip(axes, ('warmup', 'validation')):
        for g, (structure, material) in enumerate(GROUPS):
            ys = [next((r['est_error_p95_mm'] for r in rows if r['structure'] == structure and r['material'] == material
                        and r['phase'] == phase and r['condition'] == c), np.nan) for c, _ in conds]
            slot = ps.MATERIAL_STYLE[material][0]; fine = structure == 'fine'
            ax.plot(range(len(conds)), ys, marker=ps.MARKERS[slot], color=ps.color(slot), markerfacecolor=ps.SURFACE if fine else ps.color(slot),
                    markeredgecolor=ps.color(slot), linewidth=1. if fine else 1.4, label=f'{structure} {material}')
        ax.set_xticks(range(len(conds)), [l for _, l in conds], fontsize=6.8)
        ax.set_title('hands still (warm-up)' if phase == 'warmup' else 'hands moving (held-out motion)')
        ax.set_ylim(bottom=0); ax.set_xlim(-.4, len(conds)-.6)
    axes[0].set_ylabel('estimator interior error p95 (mm)')
    axes[1].legend(fontsize=6.5, loc='upper right')
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s1b_observer')


def fig_s5():
    import os
    data = Path(os.environ.get('SIM_EVAL_DATA', '/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval'))/'full'
    runs = sorted(p for p in data.glob('full_*/timeline.npz'))
    if not runs:
        return
    fig, axes = plt.subplots(2, 1, figsize=(ps.DOUBLE_COLUMN, 3.4), sharex=True)
    for n, path in enumerate(runs):
        z = np.load(path); slot = n
        releases = json.loads(str(z['releases']))
        # when the twin adopted each release: its own diagnostics report the material version it runs
        applied = {}
        for line in (path.parent/'twin.status.jsonl').read_text().splitlines():
            row = json.loads(line) if line.strip() else {}
            material = row.get('values', {}).get('material')
            if material:
                version = json.loads(material).get('version', 0)
                applied.setdefault(version, row['stamp_ns'])
        for r in releases:
            r['stamp_ns'] = applied.get(r['version'], r['stamp_ns'])
        axes[0].plot(z['t'], 1e3*z['twin_mean_error'], color=ps.color(slot), linewidth=1.,
                     label=path.parent.name.replace('full_', 'run ').replace('_r', ' r'))
        axes[1].plot(z['clearance_t'], 100*z['clearance'], color=ps.color(slot), linewidth=1.)
        for r in releases:
            for ax in axes:
                ax.axvline(r['stamp_ns']/1e9, color=ps.color(slot), linewidth=.8, linestyle=(0, (2, 2)))
            axes[0].text(r['stamp_ns']/1e9+3, 1e3*np.nanmax(z['twin_mean_error'])*.9-n*1.2,
                         f"release: bend ×{r['bend']:.2f}, shear ×{r['shear']:.2f}", fontsize=6.3, color=ps.INK2)
    for ax in axes:
        ax.axvspan(15, 93.3, color=ps.GRID, alpha=.45, linewidth=0); ax.axvspan(478, 520.8, color=ps.GRID, alpha=.3, linewidth=0)
    axes[0].text(17, .3, 'warm-up', fontsize=6.5, color=ps.INK2); axes[0].text(480, .3, 'carry', fontsize=6.5, color=ps.INK2)
    axes[1].axhline(4, color=ps.INK2, linewidth=.8); axes[1].text(100, 4.5, '4 cm margin', fontsize=6.3, color=ps.INK2)
    axes[0].set_ylabel('twin mean error (mm)'); axes[0].set_ylim(bottom=0)
    axes[1].set_ylabel('true capsule clearance (cm)'); axes[1].set_xlabel('simulation time (s)')
    axes[0].set_title('Full live loop: hidden material B, live warm-up calibration, CBF on the corrected twin')
    axes[0].legend(fontsize=6.5, loc='upper center')
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s5_full_loop')


# ------------------------------------------------------------------ S2
def fig_s2_recovery():
    rows = [r for r in read('s2_fits.csv') if 'bend' in r and isinstance(r.get('bend'), float)]
    if not rows:
        return
    fig, axes = plt.subplots(1, 2, figsize=(ps.DOUBLE_COLUMN, 3.2), sharex=True, sharey=True)
    styles = {('L0', 'extended'): ('o', True, 'L0 all-truth inputs, 13 holds'), ('L0', 'baseline'): ('o', False, 'L0, 4 holds'),
              ('L3', 'extended'): ('s', True, 'L3 live inputs, 13 holds'), ('L3', 'baseline'): ('s', False, 'L3, 4 holds')}
    for ax, structure in zip(axes, ('matched', 'fine')):
        for material, (slot, label) in ps.MATERIAL_STYLE.items():
            sub = [r for r in rows if r['structure'] == structure and r['material'] == material]
            if not sub:
                continue
            c = ps.color(slot)
            tb, ts = sub[0]['true_bend'], sub[0]['true_shear']
            ax.scatter([tb], [ts], marker='*', s=160, color=c, edgecolor=ps.INK, linewidth=.6, zorder=5)
            ax.annotate(f'θ* {material}', (tb, ts), xytext=(6, 6), textcoords='offset points', color=ps.INK, fontsize=7)
            for (level, holds), (marker, filled, _) in styles.items():
                pts = [(r['bend'], r['shear']) for r in sub if r['level'] == level and r['holds'] == holds]
                if pts:
                    b, s = np.array(pts).T
                    ax.scatter(b, s, marker=marker, s=26, color=c if filled else ps.SURFACE, edgecolor=c, linewidth=1.2, zorder=4)
        ax.set_xscale('log', base=2); ax.set_yscale('log', base=2)
        ax.set_title(f'{structure} plant' + (' (twin model class)' if structure == 'matched' else ' (56 × 56 mesh)'))
        ax.set_xlabel('bending scale')
    axes[0].set_ylabel('shear scale')
    handles = [plt.Line2D([], [], marker=m, linestyle='', markersize=6, markerfacecolor=ps.INK2 if f else ps.SURFACE,
                          markeredgecolor=ps.INK2, label=l) for (m, f, l) in styles.values()]
    handles.append(plt.Line2D([], [], marker='*', linestyle='', markersize=9, markerfacecolor=ps.MUTED, markeredgecolor=ps.INK, label='true θ*'))
    handles += [plt.Line2D([], [], marker='s', linestyle='', markersize=6, color=ps.color(s), label=l) for s, l in ps.MATERIAL_STYLE.values()]
    fig.legend(handles=handles, loc='lower center', ncol=4, bbox_to_anchor=(.5, -.12))
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s2_recovery')


def fig_s2_attribution():
    rows = [r for r in read('s2_fits.csv') if r['holds'] == 'extended' and isinstance(r.get('bend'), float) and r['repeat'] == 1]
    levels = ['L0', 'Dp', 'Gm', 'St', 'L3']
    names = {'L0': 'all truth', 'Dp': 'depth', 'Gm': 'grips', 'St': 'twin state', 'L3': 'all live'}
    if not rows:
        return
    fig, axes = plt.subplots(1, 2, figsize=(ps.DOUBLE_COLUMN, 2.4), sharey=True)
    for ax, coef in zip(axes, ('bend', 'shear')):
        for g, (structure, material) in enumerate(GROUPS):
            vals = [next((r[f'log_err_{coef}'] for r in rows if r['structure'] == structure and r['material'] == material
                          and r['level'] == lv), np.nan) for lv in levels]
            ax.plot(range(len(levels)), vals, marker=ps.MARKERS[g], color=ps.color(g), label=group_label(structure, material),
                    markeredgecolor=ps.SURFACE, markersize=5)
        ax.axhline(0, color=ps.INK2, linewidth=.8)
        ax.axhspan(-np.log(2)/8, np.log(2)/8, color=ps.GRID, alpha=.6, zorder=0)
        ax.set_xticks(range(len(levels)), [f'{lv}\n{names[lv]}' for lv in levels])
        ax.set_title(f'{coef}: log(θ̂ / θ*)')
    axes[0].set_ylabel('log(θ̂ / θ*)')
    fig.text(.5, -.02, 'Inputs swapped from plant truth to live, one at a time. Grey band: ±½ step of the bank\'s 2^(1/4) search grid.',
             ha='center', fontsize=7, color=ps.INK2)
    axes[1].legend(loc='upper left', fontsize=6.5)
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s2_attribution')


def fig_s2_landscapes():
    paths = sorted((RESULTS/'s2').glob('*.landscape.json'))
    if not paths:
        return
    data = [json.loads(p.read_text()) for p in paths]
    order = {(s, m): i for i, (s, m) in enumerate(GROUPS)}
    data.sort(key=lambda d: (order.get((d['manifest']['structure'], d['manifest']['material']), 9), d['inputs']['state'] != 'truth'))
    groups = sorted({(d['manifest']['structure'], d['manifest']['material']) for d in data}, key=lambda g: order.get(g, 9))
    fig, axes = plt.subplots(2, len(groups), figsize=(ps.DOUBLE_COLUMN, 3.4), squeeze=False)
    fits = {(r['dataset'], r['level']): r for r in read('s2_fits.csv') if r['holds'] == 'extended' and isinstance(r.get('bend'), float)}
    for d in data:
        g = groups.index((d['manifest']['structure'], d['manifest']['material']))
        row = 0 if d['inputs']['state'] == 'truth' else 1
        ax = axes[row, g]
        values = np.array(d['landscape']['values']); cost = 1e3*np.array(d['landscape']['rms_cost_m'])
        lv = np.log2(values)
        im = ax.imshow(cost.T, origin='lower', extent=(lv[0]-.5, lv[-1]+.5, lv[0]-.5, lv[-1]+.5), cmap='Blues_r', aspect='auto')
        ax.grid(False)
        t = d['manifest']['true_scales']
        ax.scatter([np.log2(t['bend'])], [np.log2(t['shear'])], marker='*', s=110, color=ps.SURFACE, edgecolor=ps.INK, linewidth=.7, zorder=5)
        i, j = np.unravel_index(np.argmin(cost), cost.shape)
        ax.scatter([lv[i]], [lv[j]], marker='o', s=26, facecolor='none', edgecolor=ps.color(1), linewidth=1.5, zorder=6)
        fit = fits.get((d['run'], 'L0' if row == 0 else 'L3'))
        if fit:
            ax.scatter([np.log2(fit['bend'])], [np.log2(fit['shear'])], marker='x', s=30, color=ps.color(7), linewidth=1.5, zorder=6,
                       clip_on=False)
        ax.set_xticks(lv[::2], [f'{v:g}' for v in values[::2]]); ax.set_yticks(lv[::2], [f'{v:g}' for v in values[::2]])
        ax.set_xlim(lv[0]-.5, 3.); ax.set_ylim(lv[0]-.5, 3.)
        if row == 0:
            ax.set_title(f'{d["manifest"]["structure"]} {d["manifest"]["material"]}', fontsize=8)
        if g == 0:
            ax.set_ylabel(('all-truth inputs' if row == 0 else 'live inputs')+'\nshear scale')
        if row == 1:
            ax.set_xlabel('bending scale')
        cb = fig.colorbar(im, ax=ax, fraction=.05, pad=.02)
        cb.ax.tick_params(labelsize=6); cb.outline.set_visible(False)
    fig.text(.5, -.03, '★ true θ*   ○ grid minimum   × bank search result      color: RMS depth cost (mm)',
             ha='center', fontsize=7, color=ps.INK2)
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s2_landscapes')


def fig_s2_adapt():
    paths = sorted((RESULTS/'s2').glob('*.adapt.json'))
    if not paths:
        return
    fig, axes = plt.subplots(2, 2, figsize=(ps.DOUBLE_COLUMN, 3.6), sharex=True)
    for row, structure in enumerate(('matched', 'fine')):
        for col, coef in enumerate(('bend', 'shear')):
            ax = axes[row, col]
            ax.axvspan(15, 93.33, color=ps.GRID, alpha=.45, linewidth=0); ax.axvspan(97.5, 140.4, color=ps.GRID, alpha=.2, linewidth=0)
            for path in paths:
                d = json.loads(path.read_text()); m = d['manifest']
                if m['structure'] != structure:
                    continue
                slot = ps.MATERIAL_STYLE[m['material']][0]
                live = d['inputs']['state'] == 'twin'
                t = [r['t'] for r in d['trace']]; v = [r[coef] for r in d['trace']]
                ax.plot(t, v, color=ps.color(slot), linewidth=1.4 if live else .8, linestyle='-' if live else ':', alpha=.9)
                ax.axhline(m['true_scales'][coef], color=ps.color(slot), linewidth=.6, alpha=.6)
            ax.set_yscale('log', base=2)
            ax.set_title(f'{structure} plant: {coef} scale', fontsize=8)
            if row == 1:
                ax.set_xlabel('time (s); shaded: warm-up, held-out motion')
    handles = [plt.Line2D([], [], color=ps.color(s), label=l) for s, l in ps.MATERIAL_STYLE.values()]
    handles += [plt.Line2D([], [], color=ps.INK2, linewidth=1.4, label='live inputs'),
                plt.Line2D([], [], color=ps.INK2, linewidth=.8, linestyle=':', label='all-truth inputs'),
                plt.Line2D([], [], color=ps.INK2, linewidth=.6, label='true value')]
    fig.legend(handles=handles, loc='lower center', ncol=6, bbox_to_anchor=(.5, -.05))
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s2_adapt')


def fig_s2_fixes():
    """Identification with live inputs: the production single-pass fit against the attempted remedies."""
    s2x = RESULTS/'s2x'
    fits = {(r['dataset'], r['level']): r for r in read('s2_fits.csv') if r['holds'] == 'extended' and isinstance(r.get('bend'), float)}
    datasets = ['id_matched_A_r1', 'id_matched_B_r1', 'id_matched_C_r1', 'id_fine_B_r1']
    methods = [('L0', 'single pass, all-truth inputs'), ('L3', 'single pass, live inputs (production)'),
               ('relax', 'live, 4.5 s free runs'), ('iterate', 'live, 3 half-step iterations'), ('long', 'live, output-error over the warm-up')]

    def value(ds, method):
        if method in ('L0', 'L3'):
            r = fits.get((ds, method)); return (r['bend'], r['shear']) if r else None
        path = {'relax': s2x/f'{ds}.L3.relax4.5.json', 'iterate': s2x/f'{ds}.iterate_r0.5.json', 'long': s2x/f'{ds}.long.L3.json'}[method]
        if not path.exists():
            return None
        d = json.loads(path.read_text())
        if method == 'iterate':
            return d['history'][-1]['bend'], d['history'][-1]['shear']
        return d['fit']['bend'], d['fit']['shear']
    reference = {'id_fine_B_r1': ('L0', 'effective θ: all-truth fit')}
    fig, axes = plt.subplots(1, 2, figsize=(ps.DOUBLE_COLUMN, 2.5), sharey=True)
    for ax, (k, coef) in zip(axes, enumerate(('bend', 'shear'))):
        for m, (method, label) in enumerate(methods):
            for d, ds in enumerate(datasets):
                v = value(ds, method)
                if v is None:
                    continue
                manifest = json.loads((Path('/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval/id')/ds/'manifest.json').read_text())
                target = (value(ds, 'L0')[k] if ds in reference else manifest['true_scales'][coef])
                x = m+(d-1.5)*.15
                slot = ps.MATERIAL_STYLE[manifest['material']][0]; fine = manifest['structure'] == 'fine'
                ax.scatter([x], [np.log(v[k]/target)], s=22, marker=ps.MARKERS[slot], color=ps.SURFACE if fine else ps.color(slot),
                           edgecolor=ps.color(slot), linewidth=1.1, zorder=3)
        ax.axhline(0, color=ps.INK2, linewidth=.8)
        ax.axhspan(-np.log(2)/8, np.log(2)/8, color=ps.GRID, alpha=.6, zorder=0)
        ax.set_xticks(range(len(methods)), ['all-truth\ninputs', 'live\n(production)', 'live\n4.5 s runs', 'live\n3 half-steps',
                                            'live\noutput error'], fontsize=6.5)
        ax.set_title(f'{coef}: log(θ̂ / θ*)')
    handles = [plt.Line2D([], [], marker=ps.MARKERS[ps.MATERIAL_STYLE[ds.split('_')[2]][0]], linestyle='', markersize=5,
                          color=ps.SURFACE if 'fine' in ds else ps.color(ps.MATERIAL_STYLE[ds.split('_')[2]][0]),
                          markeredgecolor=ps.color(ps.MATERIAL_STYLE[ds.split('_')[2]][0]),
                          label=ds.replace('id_', '').replace('_r1', '').replace('_', ' ')+(' (vs all-truth fit)' if 'fine' in ds else ''))
               for ds in datasets]
    fig.legend(handles=handles, loc='lower center', ncol=4, bbox_to_anchor=(.5, -.1), fontsize=6.5)
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s2_fixes')


# ------------------------------------------------------------------ S3
S3_MAIN = ['off_nominal', 'off_fitted', 'off_true', 'est_nominal', 'est_fitted', 'oracle_nominal']


def strip(ax, rows, conditions, key, title, ylabel):
    by = defaultdict(dict)
    for r in rows:
        by[r['dataset']][r['condition']] = (r[key], r['material'])
    for ds, vals in by.items():  # paired lines
        xs = [i for i, c in enumerate(conditions) if c in vals]
        ax.plot(xs, [vals[conditions[i]][0] for i in xs], color=ps.GRID, linewidth=.8, zorder=1)
    for i, c in enumerate(conditions):
        for material, (slot, _) in ps.MATERIAL_STYLE.items():
            v = [vals[c][0] for vals in by.values() if c in vals and vals[c][1] == material]
            ax.scatter(np.full(len(v), i)+(slot-1)*.12, v, s=18, color=ps.color(slot), marker=ps.MARKERS[slot],
                       edgecolor=ps.SURFACE, linewidth=.8, zorder=3)
        allv = [vals[c][0] for vals in by.values() if c in vals]
        if allv:
            ax.plot([i-.3, i+.3], [np.mean(allv)]*2, color=ps.INK, linewidth=1.6, zorder=4)
    ax.set_xticks(range(len(conditions)), [ps.CONDITION_STYLE[c][1].split('  ')[0] if '  ' in ps.CONDITION_STYLE[c][1]
                                           else ps.CONDITION_STYLE[c][1] for c in conditions], rotation=25, ha='right')
    ax.set_xlim(-.6, len(conditions)-.4); ax.set_ylim(bottom=0)
    ax.set_title(title); ax.set_ylabel(ylabel)


def fig_s3_state():
    rows = read('s3_replays.csv')
    if not rows:
        return
    short = {'off_nominal': 'B1\ngrip-only\nnominal', 'off_fitted': 'B2\ngrip-only\nfitted θ̂', 'off_true': 'grip-only\ntrue θ*',
             'est_nominal': 'P0\ncorrected\nnominal', 'est_fitted': 'P\ncorrected\nfitted θ̂', 'oracle_nominal': 'corrected\nby truth\nnominal'}
    for key, name, ylabel in (('validation_mean_mm', 'fig_s3_state_error', 'mean particle error (mm)'),
                              ('validation_low_abs_dz_mm', 'fig_s3_low_point', 'lowest-5 % height error (mm)')):
        fig, axes = plt.subplots(1, 2, figsize=(ps.DOUBLE_COLUMN, 2.8), sharey=True)
        top = 1.08*max(r[key] for r in rows if r['condition'] in S3_MAIN)
        for ax, structure in zip(axes, ('matched', 'fine')):
            sub = [r for r in rows if r['structure'] == structure]
            conds = [c for c in S3_MAIN if any(r['condition'] == c for r in sub)]
            strip(ax, sub, conds, key, f'{structure} plant, held-out motion', ylabel)
            ax.set_xticks(range(len(conds)), [short[c] for c in conds], rotation=0, ha='center', fontsize=6.3)
            ax.set_ylim(0, top)
        handles = [plt.Line2D([], [], marker=ps.MARKERS[s], linestyle='', color=ps.color(s), label=l) for s, l in ps.MATERIAL_STYLE.values()]
        handles.append(plt.Line2D([], [], color=ps.INK, linewidth=1.6, label='mean'))
        handles.append(plt.Line2D([], [], color=ps.GRID, linewidth=.8, label='same dataset'))
        fig.legend(handles=handles, loc='lower center', ncol=5, bbox_to_anchor=(.5, -.06))
        fig.tight_layout()
        ps.save(fig, FIGURES, name)


def fig_s3_relative():
    """Paired change of the held-out twin error against B1 (grip-only, nominal) on the same dataset."""
    rows = read('s3_replays.csv')
    if not rows:
        return
    conds = ['off_fitted', 'off_true', 'est_nominal', 'est_fitted', 'oracle_nominal']
    short = {'off_fitted': 'B2\nfitted θ̂', 'off_true': 'true θ*\n(no correction)', 'est_nominal': 'P0\ncorrection',
             'est_fitted': 'P\nfitted θ̂ +\ncorrection', 'oracle_nominal': 'correction\nby truth'}
    base = {r['dataset']: r['validation_mean_mm'] for r in rows if r['condition'] == 'off_nominal'}
    fig, ax = plt.subplots(figsize=(ps.DOUBLE_COLUMN, 2.8))
    groups = GROUPS
    width = .14
    for i, c in enumerate(conds):
        for g, (structure, material) in enumerate(groups):
            v = [100*(r['validation_mean_mm']/base[r['dataset']]-1) for r in rows
                 if r['condition'] == c and r['structure'] == structure and r['material'] == material and r['dataset'] in base]
            x = i+(g-2)*width
            slot = ps.MATERIAL_STYLE[material][0]
            filled = structure == 'matched'
            ax.scatter(np.full(len(v), x), v, s=16, marker=ps.MARKERS[slot], color=ps.color(slot) if filled else ps.SURFACE,
                       edgecolor=ps.color(slot), linewidth=1.1, zorder=3)
            if v:
                ax.plot([x-.05, x+.05], [np.mean(v)]*2, color=ps.INK, linewidth=1.2, zorder=4)
    ax.axhline(0, color=ps.INK2, linewidth=.9)
    ax.set_xticks(range(len(conds)), [short[c] for c in conds], fontsize=6.8)
    ax.set_ylabel('change vs B1 grip-only nominal (%)')
    ax.set_title('Held-out twin error relative to grip-only simulation (same dataset)')
    ax.set_xlim(-.6, len(conds)-.4)
    handles = [plt.Line2D([], [], marker=ps.MARKERS[ps.MATERIAL_STYLE[m][0]], linestyle='', markersize=5,
                          color=ps.color(ps.MATERIAL_STYLE[m][0]) if s_ == 'matched' else ps.SURFACE,
                          markeredgecolor=ps.color(ps.MATERIAL_STYLE[m][0]), label=f'{s_} {m}') for s_, m in groups]
    ax.legend(handles=handles, loc='upper left', ncol=5, fontsize=6.5)
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s3_relative')


def fig_s3_forecast():
    rows = read('s3_replays.csv')
    if not rows:
        return
    horizons = [(6, .1), (20, .33), (30, .5), (60, 1.)]
    fig, axes = plt.subplots(1, len(GROUPS), figsize=(ps.DOUBLE_COLUMN, 2.3))
    for ax, (structure, material) in zip(axes, GROUPS):
        top = 0.
        for c in ['off_nominal', 'off_fitted', 'est_nominal', 'est_fitted', 'off_true']:
            sub = [r for r in rows if r['structure'] == structure and r['material'] == material and r['condition'] == c]
            if not sub:
                continue
            slot, label = ps.CONDITION_STYLE[c]
            x = [0.]+[h for _, h in horizons]
            y = [np.mean([r['validation_mean_mm'] for r in sub])]+[np.mean([r[f'forecast_{f}_mean_mm'] for r in sub]) for f, _ in horizons]
            ax.plot(x, y, color=ps.color(slot), marker=ps.MARKERS[slot], markeredgecolor=ps.SURFACE, markersize=3.5,
                    linewidth=1.2, label=label)
            top = max(top, max(y))
        ax.set_title(f'{structure} {material}', fontsize=7.5)
        ax.set_xticks([0, .5, 1.])
        ax.set_ylim(0, 1.12*top)
    axes[0].set_ylabel('mean particle error (mm)')
    fig.text(.5, -.02, 'forecast horizon (s); 0 = current state. Open loop from each origin with the recorded future grips.',
             ha='center', fontsize=7, color=ps.INK2)
    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=5, bbox_to_anchor=(.5, -.16), fontsize=6.5)
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s3_forecast')


def fig_s3_timeseries(dataset='id_matched_B_r1'):
    files = {c: RESULTS/'s3'/f'{dataset}.{c}.npz' for c in ['off_nominal', 'off_fitted', 'est_nominal', 'est_fitted']}
    if not all(f.exists() for f in files.values()):
        return
    fig, ax = plt.subplots(figsize=(ps.DOUBLE_COLUMN, 2.2))
    ax.axvspan(15, 93.33, color=ps.GRID, alpha=.45, linewidth=0); ax.axvspan(97.5, 140.4, color=ps.GRID, alpha=.25, linewidth=0)
    ax.text(16, 1, 'warm-up (robots excite, hands still)', fontsize=6.5, color=ps.INK2, va='bottom')
    ax.text(98.5, 1, 'held-out motion', fontsize=6.5, color=ps.INK2, va='bottom')
    for c, f in files.items():
        z = np.load(f); slot, label = ps.CONDITION_STYLE[c]
        ax.plot(z['t'], 1e3*z['state_mean'], color=ps.color(slot), linewidth=1.1, label=label)
    ax.set_yscale('log'); ax.set_xlabel('time (s)'); ax.set_ylabel('mean particle error (mm)')
    ax.set_title(f'Twin error against plant truth, {dataset.replace("id_", "").replace("_", " ")}')
    ax.legend(loc='upper left', ncol=2, fontsize=6.5)
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s3_timeseries')


def fig_s3_isolation():
    rows = read('s3_replays.csv')
    if not rows:
        return
    fig, axes = plt.subplots(1, 2, figsize=(ps.DOUBLE_COLUMN, 2.4), gridspec_kw=dict(width_ratios=[1.3, 1]))
    conds = ['off_nominal', 'off_nominal_truthgrips', 'est_nominal', 'est_nominal_truthgrips', 'oracle_nominal']
    short = ['B1\nmeasured\ngrips', 'B1\ntrue\ngrips', 'P0\nmeasured\ngrips', 'P0\ntrue\ngrips', 'P0 with\ntrue-state\nobserver']
    strip(axes[0], rows, conds, 'validation_mean_mm', 'What limits the corrected twin', 'mean particle error (mm)')
    axes[0].set_xticks(range(len(conds)), short, rotation=0, ha='center', fontsize=6.5)
    ax = axes[1]
    gains = defaultdict(dict)
    for r in rows:
        if r['condition'] == 'off_nominal':
            gains[r['dataset']][0.] = r['validation_mean_mm']
        elif r['condition'].startswith('est_nominal') and r['grips'] == 'measured':
            gains[r['dataset']][r['feedback_scale']] = r['validation_mean_mm']
    for ds, g in gains.items():
        if len(g) >= 4:
            k = sorted(g); slot = ps.MATERIAL_STYLE[ds.split('_')[2]][0]; fine = ds.split('_')[1] == 'fine'
            ax.plot([max(x, .08) for x in k], [g[x] for x in k], marker=ps.MARKERS[slot], color=ps.color(slot),
                    markerfacecolor=ps.SURFACE if fine else ps.color(slot), markeredgecolor=ps.color(slot),
                    linewidth=1.0 if fine else 1.5, label=ds.replace('id_', '').replace('_r1', '').replace('_', ' '))
    ax.set_xscale('log'); ax.set_xticks([.08, .25, 1, 4], ['off', '×0.25', '×1', '×4'])
    ax.set_xlabel('correction gains'); ax.set_ylabel('mean particle error (mm)'); ax.set_title('Correction gain')
    ax.legend(fontsize=6.5, loc='center left', bbox_to_anchor=(1.0, .5))
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s3_isolation')


# ------------------------------------------------------------------ S4
def fig_s4():
    rows = read('s4_closed.csv')
    if not rows:
        return
    order = [c for c in ['B0', 'B1', 'B2', 'P0', 'P', 'G0'] if any(r['condition'] == c for r in rows)]
    specs = [('capsule_clearance_min_m', 100, 'true capsule clearance, min (cm)', 4.),
             ('margin_time_s', 1, 'time inside the 4 cm margin (s)', None),
             ('twin_error_p95_m', 1000, 'live twin error p95 (mm)', None),
             ('pull_robot_p95', 1, 'true robot-grip pull p95 (N, log)', None)]
    fig, axes = plt.subplots(1, 4, figsize=(ps.DOUBLE_COLUMN, 2.4))
    for ax, (key, scale, title, ref) in zip(axes, specs):
        for i, c in enumerate(order):
            slot, _ = ps.CLOSED_STYLE[c]
            v = [r[key]*scale for r in rows if r['condition'] == c and isinstance(r.get(key), float)]
            ax.scatter(np.full(len(v), i), v, s=22, color=ps.color(slot), marker=ps.MARKERS[slot], edgecolor=ps.INK2,
                       linewidth=.5, zorder=4)
            if v:
                ax.plot([i-.32, i+.32], [np.mean(v)]*2, color=ps.INK, linewidth=1.2, zorder=3)
        if ref is not None:
            ax.axhline(ref, color=ps.INK2, linewidth=.8); ax.text(len(order)-.6, ref, ' margin', fontsize=6, va='bottom', color=ps.INK2)
        ax.set_xticks(range(len(order)), order); ax.set_title(title, fontsize=7)
        if 'log' in title:
            ax.set_yscale('log')
        elif key != 'capsule_clearance_min_m':
            ax.set_ylim(bottom=0)
    fig.text(.5, -.04, 'B0 no CBF · B1 CBF on grip-only twin · B2 grip-only, fitted θ̂ · P0 corrected twin · '
             'P corrected, fitted θ̂ · G0 P without stretch guard   (3 repeats each; bar: mean)', ha='center', fontsize=6.5, color=ps.INK2)
    fig.tight_layout()
    ps.save(fig, FIGURES, 'fig_s4_closed_loop')


if __name__ == '__main__':
    ps.setup()
    which = set(sys.argv[1:]) or {'s1', 's2', 's3', 's4', 's5'}
    if 's1' in which:
        fig_s1(); fig_s1b()
    if 's5' in which:
        fig_s5()
    if 's2' in which:
        fig_s2_recovery(); fig_s2_attribution(); fig_s2_landscapes(); fig_s2_adapt(); fig_s2_fixes()
    if 's3' in which:
        fig_s3_state(); fig_s3_relative(); fig_s3_forecast(); fig_s3_timeseries(); fig_s3_isolation()
    if 's4' in which:
        fig_s4()
