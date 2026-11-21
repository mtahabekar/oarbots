import numpy as np, sys
from pathlib import Path
import identify_replay as ir
runs = [('live_115658', '40:120'), ('live_120408', '20:90'), ('warmup_10_0.1_111137', 'warmup')]
base = Path('experiments/hardware/sessions/2026-11-20_fabric_run')
names = ['blue', 'silver', 'right', 'left']
for run, win in runs:
    w = ir.load(base/run, win, 2, 64, 1.)
    g = w.grips
    t = (w.frame_ns-w.frame_ns[0])/1e9
    print(f'== {run} {win}: {t[-1]:.0f} s')
    for c in range(4):
        p = g[:, c]
        v = np.linalg.norm(np.diff(p, axis=0), axis=1)/np.maximum(np.diff(t), 1e-3)
        print(f'  {names[c]:>6}: range x {np.ptp(p[:,0])*100:5.1f} y {np.ptp(p[:,1])*100:5.1f} z {np.ptp(p[:,2])*100:5.1f} cm; '
              f'speed p50 {np.median(v)*100:4.1f} p95 {np.percentile(v,95)*100:4.1f} cm/s')
    rel = g[:, 2:].mean(1)-g[:, :2].mean(1)
    print(f'  hands-centre minus robots-centre: range x {np.ptp(rel[:,0])*100:.1f} y {np.ptp(rel[:,1])*100:.1f} z {np.ptp(rel[:,2])*100:.1f} cm; mean dz {rel[:,2].mean()*100:.1f} cm')
    for a, b in [(0,1),(2,3),(0,2),(1,3),(0,3),(1,2)]:
        d = np.linalg.norm(g[:, a]-g[:, b], axis=1)
        print(f'  pair {names[a]}-{names[b]}: {d.min():.3f}-{d.max():.3f} m (median {np.median(d):.3f})')
