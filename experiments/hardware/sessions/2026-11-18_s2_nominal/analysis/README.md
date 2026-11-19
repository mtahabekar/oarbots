# Cross-view checks on the S2 bag (2026-11-18, 21:50)

Bag replays with the recorded TF (`/tf`, `/tf_static`), steady hold from 2 s after the steady cue to the
movement cue (lab log, "Evening check and the S2 hand gaps"):

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_colcon_ws/install/setup.bash
bag=experiments/hardware/sessions/2026-11-18_s2_nominal/live/compact_bag
python3 experiments/hardware/sessions/2026-11-18_s2_nominal/analysis/cross_view.py  $bag 1795047542.05 1795047567.5
python3 experiments/hardware/sessions/2026-11-18_s2_nominal/analysis/glove_cross.py $bag 1795047542.05 1795047567.5
```

- `cross_view.py`: each wrist camera's sheet depth points in the world against the overhead's nearest
  capture: wrist points 3.2-4.3 cm below the overhead's, median nearest-point distance 4.6 (Blue) and
  5.8 cm (Silver).
- `glove_cross.py`: wrist glove positions against the overhead's within 70 ms, in the wrist camera
  frame: 0.5-5.5 cm along the line of sight, 3.9-14.1 cm across it.
