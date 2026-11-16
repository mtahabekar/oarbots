"""The plant's true grip pulls from its truth file, with the plant's own cloth mechanics.

    python true_tension.py RUN_DIR      (reads manifest.json for the hidden material and mesh)

For each corner patch, the resultant internal force (stretch plus bending) that the rest of the
sheet exerts on it, from the recorded particle positions: the load a wrist sensor would measure,
minus the patch's own weight and inertia. Writes RUN_DIR/true_tension.npz (times, tension (T, 4)).
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, '/home/taha/ros2_colcon_ws/src/fabric_sim_isaacsim_newton')
from capsule_cbf.cloth_model import ClothMechanics  # noqa: E402
from capsule_cbf.scenario import build_plant  # noqa: E402


def tension_series(truth_path, grid, tri_ke, edge_ke, stride=3, window=None):
    t = np.load(truth_path)
    plant = build_plant(device='cpu', grid=grid, cloth_z=1.05, tri_ke=tri_ke, edge_ke=edge_ke, capture=False)
    n = len(plant.rest)
    mech = (ClothMechanics.from_newton(plant.model, plant.model.particle_count-n, n)
            if plant.model.particle_count != n else ClothMechanics.from_newton(plant.model))
    uv = (plant.rest[:, :2]+.42)/.84

    def key(a):
        return np.round(a*grid).astype(int) @ [1, 1000]
    order = np.argsort(key(t['material_uv']))
    inverse = np.empty(n, int); inverse[np.argsort(key(uv))] = np.arange(n)
    to_model = order[inverse]  # truth index of each model particle
    patches = [plant.patch[plant.corner_ids == c] for c in range(4)]
    times, out = [], []
    frames = range(0, len(t['times_ns']), stride)
    if window is not None:  # only the scored window (with 2 s either side)
        seconds = t['times_ns']/1e9
        frames = [k for k in frames if window[0]-2 <= seconds[k] <= window[1]+2]
    for k in frames:
        q = t['positions'][k][to_model].astype(float)
        out.append([np.linalg.norm(mech.resultant(q, p)) for p in patches])
        times.append(t['times_ns'][k]/1e9)
    return np.array(times), np.array(out)


if __name__ == '__main__':
    run = Path(sys.argv[1])
    manifest = json.loads((run/'manifest.json').read_text())
    b, s = manifest['true_scales']['bend'], manifest['true_scales']['shear']
    grid = 28 if manifest.get('structure') == 'matched' else 56
    times, tension = tension_series(str(run/'truth.npz'), grid, (3e4, 3e4, 6e3*s), (.002*b, .001*b, .0005*b),
                                    window=manifest.get('carry_window_s'))
    np.savez(run/'true_tension.npz', times=times, tension=tension)
    print(run.name, 'robot pull p95', np.round(np.percentile(tension[:, :2], 95), 2), 'N')
