"""Figure style for the sim_eval report and paper (static matplotlib, PDF + PNG).

Palette: the dataviz reference categorical order (blue, orange, aqua, yellow, magenta, green,
violet, red), assigned to entities in a fixed order and never cycled. Every figure also encodes
identity by position, marker shape or a direct label, so no reading depends on color alone.
Text uses text inks, never series colors; grid and axes are hairline and recessive.
"""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

SURFACE, INK, INK2, MUTED, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984', '#e4e3df'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']
MARKERS = ['o', 's', 'D', '^', 'v', 'P', 'X', 'h']
SINGLE_COLUMN, DOUBLE_COLUMN = 3.5, 7.16   # IEEE column widths, inches

# Fixed entity -> slot assignments used across all figures.
CONDITION_STYLE = {   # twin conditions (S3) and closed-loop conditions (S4)
    'off_nominal': (0, 'B1  grip-only, nominal θ'),
    'off_fitted': (1, 'B2  grip-only, fitted θ̂'),
    'off_true': (5, 'grip-only, true θ*'),
    'off_l0fit': (3, 'grip-only, θ fitted on true inputs'),
    'est_nominal': (2, 'P0  corrected, nominal θ'),
    'est_fitted': (6, 'P   corrected, fitted θ̂'),
    'oracle_nominal': (4, 'corrected by truth, nominal θ'),
    'off_nominal_truthgrips': (7, 'B1 with true grips'),
    'est_nominal_truthgrips': (4, 'P0 with true grips'),
}
CLOSED_STYLE = {'B0': (3, 'B0  no CBF'), 'B1': (0, 'B1  CBF, grip-only twin'), 'B2': (1, 'B2  CBF, grip-only, fitted θ̂'),
                'P0': (2, 'P0  CBF, corrected twin'), 'P': (6, 'P   CBF, corrected, fitted θ̂'), 'G0': (7, 'G0  P without guard')}
MATERIAL_STYLE = {'A': (0, 'A  (×1, ×1)'), 'B': (1, 'B  (×3, ×0.4)'), 'C': (2, 'C  (×1/3, ×2.5)')}


def setup():
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'font.size': 8, 'axes.titlesize': 8.5, 'axes.labelsize': 8, 'xtick.labelsize': 7, 'ytick.labelsize': 7,
        'legend.fontsize': 7, 'legend.frameon': False, 'font.family': 'DejaVu Sans',
        'text.color': INK, 'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
        'axes.edgecolor': GRID, 'axes.linewidth': .6, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': .5,
        'grid.linestyle': '-', 'axes.axisbelow': True, 'axes.spines.top': False, 'axes.spines.right': False,
        'lines.linewidth': 1.5, 'lines.solid_capstyle': 'round', 'lines.solid_joinstyle': 'round',
        'lines.markersize': 4.5, 'xtick.major.size': 0, 'ytick.major.size': 0, 'axes.titleweight': 'bold',
        'axes.titlelocation': 'left', 'pdf.fonttype': 42, 'ps.fonttype': 42})


def color(slot):
    return SERIES[slot % len(SERIES)]


def save(fig, out_dir, name):
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out/f'{name}.pdf', bbox_inches='tight')
    fig.savefig(out/f'{name}.png', dpi=220, bbox_inches='tight')
    plt.close(fig)
    return out/f'{name}.png'
