# fabric_twin

A simulated copy of the sheet that runs beside the real cell. Its four grips follow the measured
robot and glove grips, the estimator corrects it, and the sensitivity CBF runs on it against a
simulated obstacle. See `docs/fabric-twin.md` in the repository for the design, the commands and
the measured results.

Extended identification motions and comparison procedure are documented in
`../fabric_sim_cell/README.md` (Extended material-identification warm-up).
Run CPU trajectory checks with:

```bash
PYTHONPATH=src/fabric_twin ~/venvs/fabric_ros/bin/python -m pytest -q src/fabric_twin/test/test_warmup.py
```

(`docs/fabric-twin.md`, referenced above, is not in the repository as of 2026-11-19.)

## Obstacle

To capture, refit or select a physical object for the next twin launch, follow the
[hardware obstacle workflow](../../scripts/hw/README.md#3-capture-and-select-a-real-obstacle).

The twin's obstacle comes from `twin.yaml` (`obstacle:` a box: center, half_extents, yaw; the CBF uses capsule_cbf's
circular capsule around it) unless the node gets `obstacle_file` (the compute launch passes `cell_dir/obstacle.yaml`
when it exists; `scripts/hw/fabric_run.sh obstacle use|clear` manages it). That file's `obstacle:` is either a box or a detected
object (`shape: prism_capsule`, written by `scripts/hw/obstacle_detect.py` from depth; `fabric_twin/obstacle_shape.py`):

- `prism`: the rough shape, an n-sided footprint (18 by default: 20 faces) from the floor to `top`. Sides: the points'
  minimum-area rectangle, then the convex-hull edges' support lines that cut away the most area, up to n; each moved
  out by the pad. The twin sheet collides with it (Newton mesh shape; the CBF's coarse model copies meshes since
  capsule_cbf's material_grid change of 2026-11-19).
- `capsule`: capsule_cbf's own capsule (vertical cylinder from the floor, spherical cap through its rim; center,
  radius, cylinder_height, cap_sphere_radius) fitted to the prism: smallest volume containing every prism corner
  (SLSQP). `capsule_args` turns it into `Capsule(center, half_extents, 'centered_sphere', top_radius_scale)`; the CBF
  and `TwinCloth.obstacle_clearance` use that, unchanged. For a flat-topped prism the smallest cap is nearly flat.
- RViz draws the prism (orange) and the capsule (blue) instead of the box and its envelope.
- A virtual collision (clearance below `runtime.collision_clearance_m`) is reported in the status
  (`virtual_collision`) but no longer zeroes the commands (2026-11-19).

```bash
PYTHONPATH=src/fabric_twin ~/venvs/fabric_ros/bin/python -m pytest -q src/fabric_twin/test/test_obstacle_shape.py
```
