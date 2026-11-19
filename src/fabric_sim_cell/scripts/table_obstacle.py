#!/usr/bin/env python3
"""The simulated cell's table as a detected obstacle: obstacle.yaml with its prism and fitted capsule.

The plant always contains the table, so the simulated cameras cannot give an empty background for
scripts/hw/obstacle_detect.py. This builds the same description from the table's exact geometry (oarbot_simulation
config/fabric_layout.json): points on its surface -> fabric_twin.obstacle_shape prism (up to --sides sides, --pad
out) and the capsule fitted to it, written in obstacle_detect.py's format (shape prism_capsule).
    python3 table_obstacle.py OUT.yaml [--sides 18] [--pad 0.01]      (OARBOT_SIM: path to oarbot_simulation)
"""
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'fabric_twin'))
from fabric_twin.obstacle_shape import fit_capsule, prism_footprint  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('out')
    ap.add_argument('--sides', type=int, default=18)
    ap.add_argument('--pad', type=float, default=0.01)
    a = ap.parse_args()
    sim = Path(os.environ.get('OARBOT_SIM', '/media/taha/ExtermeLinux/oarbot_simulation'))
    layout = json.loads((sim / 'config' / 'fabric_layout.json').read_text())
    hx, hy, hz = layout['table_half_extents']            # centred on the world origin, standing on the floor
    g = np.linspace(-1, 1, 21)
    top = np.array([(x*hx, y*hy) for x in g for y in g])
    foot = prism_footprint(top, sides=a.sides, pad=a.pad)
    height = 2*hz+a.pad
    capsule = fit_capsule(foot, height)
    result = dict(obstacle=dict(shape='prism_capsule',
                                prism=dict(sides=int(len(foot)), top=round(height, 4),
                                           footprint=[[round(float(x), 4), round(float(y), 4)] for x, y in foot]),
                                capsule=capsule),
                  source=dict(table_half_extents=[hx, hy, hz], layout=str(sim / 'config' / 'fabric_layout.json'),
                              pad_m=a.pad, note='from the plant geometry, not from depth'))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text('# The simulated table as a detected obstacle (src/fabric_sim_cell/scripts/table_obstacle.py).\n'
                           + yaml.safe_dump(result, sort_keys=False, default_flow_style=None))
    print(f'table {2*hx:.2f} x {2*hy:.2f} x {2*hz:.2f} m -> prism {len(foot)} sides to {height:.3f} m; capsule radius '
          f'{capsule["radius"]:.3f} m, cylinder {capsule["cylinder_height"]:.3f} m, cap {capsule["cap_height"]:.3f} m, '
          f'{capsule["volume_m3"]*1000:.0f} L -> {a.out}')


if __name__ == '__main__':
    main()
