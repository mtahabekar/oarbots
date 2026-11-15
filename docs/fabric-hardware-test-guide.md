# Fabric pipeline: pre-hardware work, rehearsal results, and test-day guide

**Date:** 13 November 2026. Follows the [hardware demonstration plan](fabric-hardware-demo-plan.md) (work items 4.1–4.7).

**Evidence boundary.** Everything below was built and measured on the compute laptop (RTX 5070 Laptop GPU) against the simulated cell in Isaac Sim 6.2 and against recordings of it. Nothing ran on the robots, the Azure cameras, or real carbon fabric. Numbers are single runs unless a range is given, and replays vary by several millimetres from run to run. Nothing is committed.

## 1. What now exists

| Plan item | Status | Where |
| --- | --- | --- |
| 4.1 Estimator fixes | Done; verified on the three replayed teleoperation failures | `fabric_state_estimator`, `human_hand_tracking`, `fabric_state_msgs` |
| 4.2 Real-cell deployment | Launch files and configuration **templates**; values must be measured on site | `src/fabric_cell` |
| 4.3 Twin node | Done; runs headless at 60 Hz, 6.5 ms per frame alone | `src/fabric_twin` |
| 4.4 Estimate-to-twin feedback | Done; bounded force with a learned residual term | `fabric_twin/feedback.py` |
| 4.5 Warm-up calibration, online adaptation | Calibration rehearsed live; online update tested headless only | `fabric_twin/bank.py`, `bank_node.py`, `warmup.py` |
| 4.6 CBF command path | Guard mode, bridge with latching stops, GUI toggle; logic tested, **not run on a robot** | `oarbot_runtime`, `tablet_gui` |
| 4.7 Closed-loop rehearsal | Done in Isaac: robots driven only by the twin's commands | `fabric_sim_cell` scripts, `oarbot_simulation --external-followers` |

Tests: **100 pass** (fabric packages, twin, guard and bridge logic). The tablet GUI change is untested here because PySide6 is not installed on this machine.

## 2. Measured results

### 2.1 Estimator on the three teleoperation failures

The original runs kept only plant truth, so each was rebuilt: the recorded hand motion was replayed into the plant (hand path reproduced within 1 mm) and the sensor topics were recorded. The unmodified estimator reproduced the failure; the recordings were then replayed at real-time rate through the changed estimator.

| Clip | Surface p95 before / after | Human-corner p95 before / after | Robot-corner p95 before / after | 95% coverage before / after |
| --- | --- | --- | --- | --- |
| `222046` | 54.7 / 24.1 mm | 115 / 8–15 mm | 14–18 / 0.3 mm | 0.74 / 0.92 |
| `223543` | 66.7 / 44.5 mm | 79–144 / 5–18 mm | 16–18 / 0.5 mm | 0.63 / 0.84 |
| `223917` | 77.4 / 32.7 mm | 109–119 / 18–31 mm | 16–17 / 0.4 mm | 0.61 / 0.81 |

**The cause was not what the progress report expected.** The glove estimate reaching the estimator was already accurate: 2 mm typical and 4–7 mm p95 against the true hand position, including propagation. The error came from the camera update itself, which moved the human corners 4–15 cm away from that estimate (the glove hides the sheet's corner). The fix that worked:

- **All four corners are prescribed.** Camera evidence never moves a corner. Robot grips come from TF; human grips from the hand filter.
- **Human grips are carried to the capture time with the hand filter's own model**, interpolated when a later glove detection has already arrived (`HandTrack`). `HandState` now carries acceleration, the 9×9 covariance and the jerk density.
- **A missed tag no longer drops the analysis.** The grip coasts for `hand_coast_sec`, then holds.
- **Grip perturbations are centred and whitened**, and an uncertain grip's spread is scaled about its mean so no member overstretches the sheet.
- **Relaxation to prior spread (0.5)** for coverage. Values of 0.8–1.0 covered more but cost up to 15 mm of p95.

Two designs were tried and rejected first: human grips as ensemble state (diverged: 16 members cannot track velocity from 5 Hz detections), and a bounded camera correction of the corners (the correction was the error).

### 2.2 Live closed loop in Isaac

Plant with gloves seen at 15 Hz as on the real cell. The human hands carry the sheet down toward the obstacle, sideways across it, and back up. The simulated robots receive only the twin's commands.

| Run | True clearance from the obstacle capsule | True clearance from the box | Twin error vs true sheet (p50 / p95) |
| --- | --- | --- | --- |
| No filter | **−5.9 cm** (sheet touches the box) | 0.0 cm | 2.3 / 17.3 mm |
| Twin CBF, 5 cm margin | 4.3 cm | 8.0 cm | 6.3 / 20.4 mm |
| Twin CBF, 8 cm margin, after warm-up calibration | **5.7 cm** | 10.4 cm | 3.9 / 17.3 mm |

The live estimator in these runs: p50 4–8 mm, p95 15–26 mm, coverage 0.95–0.97.

### 2.3 Warm-up calibration

The plant was given different coefficients (bending ×3, shear ×0.4). The robots moved their grips 2 and 4 cm toward the hands and held; the bank fitted the twin's two scales to the cameras' depth points at the holds.

- **Shear:** fitted ×0.42 (true ×0.40).
- **Bending:** fitted ×6.7 (true ×3). The plant uses a 57×57 mesh and the twin 29×29, and bending is weakly identified; treat the bending value as an effective number for the twin's mesh.
- **Depth residual at the holds:** 8.0 mm before, 6.1 mm after.
- **Time:** 245 s with Isaac sharing the GPU; about 100 s headless.

Headless, with the same mesh on both sides, the fit recovered (3.0, 0.4) as (3.36, 0.42) and (0.5, 2.0) as (0.42, 1.68).

## 3. Problems found that you should know about

- **Tension is the weak point.** The plant's true grip tension reached 60–70 N during the carry, against a 20 N policy limit, and the twin's CBF reported its tension and strain limits relaxed on about three quarters of its updates. A sheet held exactly to its width turns every centimetre of following lag into tens of newtons; the first rehearsal stretched the true sheet by 17 mm. The node now keeps 4 cm of slack toward the hands and 2 cm sideways, which reduced it but did not remove it. On hardware, **the wrist-force stop in the bridge is the only real tension protection**; set its limit conservatively and keep the sheet visibly slack.
- **The twin does not run at 60 Hz beside Isaac.** Frame time was 16 ms median and 30 ms p95 with Isaac, the estimator and the twin on one 8 GB GPU; alone it is 6.5 ms. The real cell has no Isaac, but the estimator shares the GPU, so measure it there (gate H12).
- **A slack sheet is not predictable from coefficients alone.** Two identical simulations of a sheet with 6 cm of slack drift 20 mm apart in 10 s. With 3 cm they stay within 0.05 mm. Keep warm-up insets small (2–4 cm), which is why the defaults changed from the plan's 3/6/9 cm.
- **Feedback narrows a model error but does not close it.** With coefficients wrong by ×3 and ×0.4 on a slack sheet, the corrective force brought the twin from 77 mm to 59 mm of error. Calibration is what removes it.
- **Online adaptation is unproven live.** `Bank.adapt` is tested headless (it moves toward the better coefficient and refuses to act on a taut sheet). It has not been run in the live loop; it is off by default in the simulated-cell launch and set to every 10 s in `fabric_cell`. Decide after watching it in a shadow session.
- **`grip measurement is missing` appeared on 5–13% of CBF updates** in the live runs, each of which sends zero to the robots. I did not find out whether the cause is glove dropouts or TF timing.

## 4. Running it

Build with the environment that has Newton 1.6.1 (`~/venvs/fabric_ros`):

```bash
cd ~/ros2_colcon_ws && source /opt/ros/jazzy/setup.bash
~/venvs/fabric_ros/bin/python -m colcon build --packages-select fabric_state_msgs
source install/setup.bash
~/venvs/fabric_ros/bin/python -m colcon build --packages-select fabric_perception human_hand_tracking \
    fabric_state_estimator fabric_reference_reconstruction fabric_sim_cell fabric_twin fabric_cell
```

Simulated cell (scripts in `src/fabric_sim_cell/scripts`):

```bash
# Repeat a recorded run with replayed hand motion, record the sensor topics
run_teleop_replay.sh TRUTH.npz OUT_DIR gloves
# Replay that recording through the estimator (and twin) without Isaac
replay_bag.sh OUT_DIR NEW_OUT "hands:=gloves profile:=fast twin:=true twin_record:=NEW_OUT/twin.npz"
# Closed loop: the plant's robots take the twin's commands
run_teleop_replay.sh SCRIPT.npz OUT_DIR gloves \
    "twin:=true twin_commands:=true twin_bank:=true twin_record:=OUT_DIR/twin.npz" \
    "--external-followers --overhead-hz 15"
# A two-hand carry script for the plant
ros2 run fabric_sim_cell leader_script SCRIPT.npz --gather 0.02 --start 14 --move 0 0 -0.55 0.08 --hold 3 ...
```

Real cell, after copying `src/fabric_cell/config` to a directory and measuring its values:

```bash
ros2 launch fabric_cell robot_host.launch.py robot:=blue cell_dir:=/abs/cell      # each helper
ros2 launch fabric_cell camera_host.launch.py camera:=overhead cell_dir:=/abs/cell # NUC
ros2 launch fabric_cell compute.launch.py cell_dir:=/abs/cell commands:=false      # this laptop, shadow
```

Operating the twin:

| Action | Command |
| --- | --- |
| Start the warm-up excitation | `ros2 topic pub --once /fabric/twin/warmup std_msgs/String "{data: start}"` |
| Enable or disable robot commands | `ros2 topic pub --once /fabric/twin/enable_commands std_msgs/Bool "{data: true}"` |
| Arm a robot's bridge (takes the force baseline) | `ros2 topic pub --once /oarbot_blue/fabric_cbf/arm std_msgs/Bool "{data: true}"` |
| Select the guard mode | "Fabric Follow (CBF)" toggle in the tablet GUI, deadman held |
| Watch | `/fabric/twin/status`, `/fabric/twin/cbf_debug`, `/fabric/bank/status`, `/fabric/material_parameters`, RViz `/fabric/twin/markers` |

The obstacle is the `obstacle` section of `twin.yaml` (centre, half extents, yaw in the world frame). The twin sends zero whenever a grip measurement is missing, the estimator is stale, the CBF fails, or the simulated sheet touches the obstacle.

## 5. What is left for you, in order

1. **On this laptop (needs `sudo`):** install `ros-jazzy-rmw-cyclonedds-cpp` and `chrony`; attach a USB Ethernet adapter and put its interface name in a copy of `config/cyclonedds_wired.xml`.
2. **On the helpers:** build `oarbot_runtime` and `tablet_gui` with the new guard mode, bridge and GUI toggle. Neither could be built or run here (no `kinova_msgs`, no PySide6). Rebuild `fabric_state_msgs` everywhere: `HandState` changed.
3. **Fill the `fabric_cell` templates:** sheet size and mass, tag size and tag-to-pinch offset, colour thresholds, `calibration.yaml` (the launch refuses to start while it says `measured: false`), workspace box and force limit in `bridge.yaml`, obstacle in `twin.yaml`.
4. **Hardware gates** from the plan (§5): carbon depth first, then H1–H3, H5, H7, H9, H12, H15. For H3, run the bridge with a 2 cm/s command along each world axis and confirm the arm moves that way; the Kinova command frame in `bridge.yaml` is an assumption.
5. **Shadow sessions before any closed loop:** estimator alone, then twin with `commands:=false`, then warm-up, then closed loop without an obstacle.

## 6. Changes outside this repository

`/media/taha/ExtermeLinux/oarbot_simulation/scripts/run_oarbot_fabric.py` gained `--external-followers` and `--external-timeout` (the plant's robots take grip velocities from `/oarbot_{blue,silver}/fabric_cbf/cmd_world`). That checkout already had uncommitted changes of yours; mine are additive. Rehearsal outputs are under `logs/sim_cell/hwprep/` there (about 2 GB).
