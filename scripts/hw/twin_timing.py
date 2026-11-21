#!/usr/bin/env python3
"""Time the live twin's frame at candidate numerics (`fabric_run.sh twin-timing [mesh:substeps:iterations ...]`).

The twin must step a 60 Hz frame (16.7 ms) and leave room for the CBF (about 7.5 ms every 4th frame). This builds
the real TwinCloth (CUDA graph, point grips, self-collision off unless --self-collision) for each candidate and
times 600 frames of a slow sinusoidal grip motion after a 120-frame warm-up. Run it alone on the GPU: other GPU
jobs make it slower than the live twin.

Candidates are mesh:substeps:iterations, e.g. 20:32:10. Default: the deployed 28:8:6 and the converged candidates.
"""
import argparse
import dataclasses
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/fabric_twin'))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('candidates', nargs='*', default=['28:8:6', '14:16:10', '14:32:10', '20:16:10', '20:32:10', '28:32:10'])
    ap.add_argument('--config', default=str(REPO / 'experiments/hardware/cell/twin.yaml'))
    ap.add_argument('--self-collision', action='store_true')
    ap.add_argument('--frames', type=int, default=600)
    a = ap.parse_args()
    import warp as wp
    from fabric_twin.cloth import Obstacle, TwinCloth
    from fabric_twin.node import load_twin_config
    base = load_twin_config(Path(a.config))[0]
    grips = np.array([[-.39, -.42, 1.05], [.38, -.43, 1.06], [-.33, .25, 1.08], [.41, .21, 1.03]])
    print(f'{"candidate":>12} {"ms/frame":>9} {"p95":>6}  budget 16.7 ms (60 Hz), CBF adds ~7.5 ms every 4th frame')
    for c in a.candidates:
        cells, substeps, iterations = (int(x) for x in c.split(':'))
        spec = dataclasses.replace(base, cells_x=cells, cells_y=cells, substeps=substeps, iterations=iterations,
                                   self_collision=a.self_collision, grip_insets_m=base.grip_insets_m or (0., 0., 0., 0.),
                                   obstacle=Obstacle(center=(0., 0., -50.), half_extents=(.1, .1, .1)))
        twin = TwinCloth(spec, grips)
        twin.place(grips, .03)
        times = []
        for k in range(120+a.frames):
            g = grips+np.array([0., .03*np.sin(k/40), .02*np.sin(k/55)])
            t0 = time.perf_counter()
            twin.step(g)
            wp.synchronize()                            # count the GPU work, not only the launch
            times.append(time.perf_counter()-t0)
        ms = np.array(times[120:])*1e3
        print(f'{c:>12} {ms.mean():9.2f} {np.percentile(ms, 95):6.2f}', flush=True)


if __name__ == '__main__':
    main()
