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
| Twin CBF, 5 cm margin (superseded) | 4.3 cm | 8.0 cm | 6.3 / 20.4 mm |
| Twin CBF, 8 cm margin, after warm-up calibration (superseded) | 5.7 cm | 10.4 cm | 3.9 / 17.3 mm |
| **Current settings** (stretch guard, clearance-only CBF, 4 cm margin) | 3.9 cm | **8.2 cm** | 4.4 / 20.8 mm |

The first three runs used CBF settings that stretched the sheet to hold clearance; §3 explains why they were replaced.

The live estimator in these runs: p50 4–8 mm, p95 15–26 mm, coverage 0.95–0.97.

### 2.3 Warm-up calibration

The plant was given different coefficients (bending ×3, shear ×0.4). The robots moved their grips 2 and 4 cm toward the hands and held; the bank fitted the twin's two scales to the cameras' depth points at the holds.

- **Shear:** fitted ×0.42 (true ×0.40).
- **Bending:** fitted ×6.7 (true ×3). The plant uses a 57×57 mesh and the twin 29×29, and bending is weakly identified; treat the bending value as an effective number for the twin's mesh.
- **Depth residual at the holds:** 8.0 mm before, 6.1 mm after.
- **Time:** 245 s with Isaac sharing the GPU; about 100 s headless.

Headless, with the same mesh on both sides, the fit recovered (3.0, 0.4) as (3.36, 0.42) and (0.5, 2.0) as (0.42, 1.68).

## 3. Tension: what limits it, and how it is controlled

**The 20 N limit had no basis and is gone.** It was the default of the simulator's CBF. Two problems:

- **The value itself.** Nothing about this cell supports 20 N.
- **The quantity behind it.** The tension was predicted by the twin's sheet model, from a stretch stiffness that is uncalibrated and that the warm-up cannot identify (a taut sheet's shape does not depend on it).

**What happened with it.** In rehearsal the CBF learned a safety margin on tension from its own prediction errors. That margin grew to 28–30 N, larger than the limit, so the tension rows were infeasible and relaxed on most updates. The cause is that the twin's robot corners follow the measured arms, which lag the commands, and near tautness 1 mm of error is about 90 N in the model. The CBF then kept obstacle clearance by spreading and raising the robots until the sheet was taut. Robot-to-robot distance reached 0.83–0.85 m on a 0.84 m sheet, and the true grip pull reached 60 N.

### 3.1 What actually limits the pull at a grip

| Limit | Estimate | Basis | Status |
| --- | --- | --- | --- |
| Arm payload margin | roughly 3–6 N per arm | Kinova rates the Jaco2 6-DOF 1.6 kg continuous at mid-range and 1.3 kg at full reach ([product page](https://assistive.kinovarobotics.com/product/jaco-robotic-arm), [2018 specification sheet](https://brussels.autonomia.org/pdf/docs/605/KINO-2018-TechSpecs-07-Jaco6DS-EN-Web.pdf)). In the robot model, joint 6 carries the 0.99 kg stock hand plus 0.95 kg added by this cell: wrist sensor stack 0.26 kg, camera bracket and connectors 0.25 kg, Azure camera 0.44 kg. | Datasheet plus model, assuming the rating is beyond the stock hand. **Measure it.** |
| Robot grip slip on dry carbon | unknown | Depends on finger force, finger material and the ply. No published value found for the KG-3 on dry carbon. | **Measure it.** |
| What the person holds comfortably | probably 10 N or less | Tip-pinch strength is about 70–110 N maximum in adults ([Mathiowetz et al. 1985](https://pubmed.ncbi.nlm.nih.gov/3970660/)). A sustained comfortable fraction is far lower; my estimate, not a sourced figure. | **Ask and measure.** |
| Fibre damage | not the limit | Dry tows are strong in tension. The risk is distortion: pre-tension must run along the fibres, because off-axis pulls distort woven fabric ([review of automated ply handling](https://www.tandfonline.com/doi/full/10.1080/20550340.2020.1750329)). | Protected by geometry, not a force number. |

The literature on human-robot sheet co-manipulation does not use a fixed tension limit. It either regulates a small measured tension with force control ([Kruse et al., ICRA 2015](https://sites.ecse.rpi.edu/~rjradke/papers/kruse-icra15.pdf), from this lab) or keeps the grips slightly closer than the ply's length ([Ojanen et al. 2026](https://arxiv.org/abs/2606.11818)).

### 3.2 The approach now

1. **Geometry first: the stretch guard** (`fabric_twin/stretch.py`, `stretch_guard` in `twin.yaml`). Every grip pair through a robot stays at least 3 cm shorter than the sheet between them: the two robot-to-hand edges, the robot-to-robot edge, and the two diagonals. It uses only measured positions (robot TF and the glove filter) and runs after the CBF, with priority over clearance. In this slack regime the pull at a grip is set by the sheet's weight and motion, not by the unknown stretch stiffness. The 3 cm must exceed the grip position error plus arm speed × latency; recompute it from the measured errors.
2. **The CBF does clearance only** (`fabric_twin/config/cbf.json`). Tension and strain rows are removed; the strain limits (2% stretch, 0.2 shear cosine) had no basis either and added nothing measurable once the guard was in place. Arm acceleration is limited to 0.4 m/s² instead of 3 m/s², close to the 0.3 m/s² the CBF assumes for the human. Abrupt clearance corrections were the main source of the remaining pull peaks: at 1 m/s² the p95 pull doubled.
3. **The clearance margin is derived**, not inherited: 4 cm, which is the twin's error against the real sheet (about 2 cm p95) plus 0.1 m/s × 0.15 s. The capsule envelope already lies 7.3 cm above the box top at its centre.
4. **The measured wrist force is the backstop.** It has two thresholds, both to be set from the measured capacities in §3.1:
   - `force_soft_n` in the guard: the robot yields along the cloth's pull. Off until gates H5/H6 pass. Placeholder 3 N.
   - `force_limit_n` in the bridge: the arm stops. Placeholder 6 N, the upper end of the estimated arm margin.

   A suggested rule: stop at the smallest of the arm margin, half the robot grip slip force, and what the person holds comfortably; yield at half of that.

### 3.3 Results

| Run (same carry, hands to 0.5 m) | Robot-grip pull p50 / p95 / p99 / max | Hand-grip pull p95 / max | Robot grips above 6 N | Clearance from box |
| --- | --- | --- | --- | --- |
| Before: CBF with 20 N tension and strain rows, no guard | 4.2 / 23.4 / 36.9 / 60.2 N | 15.1 / 48.6 N | 36% of the time | 8.0 cm |
| **Now:** guard, clearance-only CBF, 0.4 m/s², 4 cm margin | 2.5 / 6.9 / 12.9 / 23.5 N | 4.4 / 30.0 N | 7% | 8.2 cm |
| No CBF (sheet touches the obstacle) | 1.3 / 3.9 / 4.7 / 6.3 N | 3.2 / 5.0 N | 0% | 0.0 cm |
| Now, plus force yielding above 3 N (two-way grip forces, simulated wrist sensors) | 2.5 / 7.2 / 11.6 / 13.1 N | 5.8 / 13.5 N | 8% | 7.9 cm |

These are Isaac runs, with the true pull computed from the plant's own 57×57 cloth mechanics, independent of the twin's 29×29 mesh. They use the simulator's uncalibrated material and a 0.14 kg sheet. Read them as comparisons, not as hardware forces. Single-frame maxima are unreliable in the slack regime: a crumpling corner gives one-frame spikes, for example 4 → 105 → 22 N within 0.1 s in a headless run, while every grip distance was shrinking.

The force-yield run also checks the hardware force path end to end: simulated wrist sensor, compensator, rotation into the world at the sample time, and the sign convention (`force_on_robot_sign: +1`). The sensors read 2.3 N median and 6.7 N p95 of cloth pull on each robot, close to the plant's mechanics. Yielding cut the single-frame peaks (13 N against 23.5 N) but not the p95, in one run each.

Most of the remaining time above 6 N happens when the guard has no fresh hand measurement and stops the robots while the hands keep moving. The CBF needed no hard-row relaxation on any update (472 feasible, 171 soft-relaxed for clearance).

**Hand height is the lever.** Headless, with the current settings:

| Hands lowered to | Robot-grip pull p95 | Box clearance | What the CBF does |
| --- | --- | --- | --- |
| 0.65 m | 2.8 N | 16 cm | nothing; the sheet clears on its own |
| 0.60 m | 2.8 N | 11 cm | nothing; the 4 cm margin is already met |
| 0.55 m | 4.3 N | 10 cm | changes the command on 52% of updates; cannot hold the full margin on 28% |
| 0.50 m | 4.1 N | 8 cm | 57%; 32% |

Repeated headless runs vary by about 2 N in p95, so 0.55 m and 0.50 m are not distinguishable.

Holding the sheet clear costs pull, and the lower the hands go over the obstacle, the more it costs. For the demo, the person should go low enough that the CBF visibly acts, and no lower than the measured force limit allows.

### 3.4 What to measure on site

1. **Sheet mass.** Weigh it; this sets the static pull.
2. **Arm margin.** At the grasp pose, with the full tool, hang known masses (0.2, 0.4, 0.6 kg) from the grip, then pull horizontally toward the person with a spring scale. Watch the arm's joint efforts against Kinova's actuator ratings and the position hold. Take the force where the arm still holds with margin.
3. **Robot grip slip.** Close the fingers on a corner as in the demo and pull the sheet along the edge and along the diagonal while recording the wrist force. The slip force is the peak before the corner slides; repeat five times.
4. **The person.** Hold a corner with the tagged glove for two minutes at increasing pulls; note the comfortable level and the slip level.
5. **Grip position error.** Robot grip calibration residual and glove tag-to-pinch error. These set the guard's slack.

Then set `force_limit_n` (bridge), `force_soft_n` (guard, after H5/H6) and `edge_slack_m`/`diagonal_slack_m` from those numbers.

## 4. Problems found that you should know about

- **Glove dropouts stop the robots.** When a hand has not been seen for `hand_coast_sec` (1 s), the guard sends zero. Commands were zero on 5–9% of updates in the latest rehearsals. While the hands keep moving, that is also the main source of the remaining pull peaks. The cause is glove visibility, not the controller.
- **The twin does not run at 60 Hz beside Isaac.** Frame time was 16 ms median and 30 ms p95 with Isaac, the estimator and the twin on one 8 GB GPU; alone it is 6.5 ms. The real cell has no Isaac, but the estimator shares the GPU, so measure it there (gate H12).
- **A slack sheet is not predictable from coefficients alone.** Two identical simulations of a sheet with 6 cm of slack drift 20 mm apart in 10 s; with 3 cm they stay within 0.05 mm. Keep warm-up insets small (2–4 cm), which is why the defaults changed from the plan's 3/6/9 cm.
- **Feedback narrows a model error but does not close it.** With coefficients wrong by ×3 and ×0.4 on a slack sheet, the corrective force brought the twin from 77 mm to 59 mm of error. Calibration is what removes it.
- **Online adaptation is unproven live.** `Bank.adapt` is tested headless: it moves toward the better coefficient and refuses to act on a taut sheet. It has not run in the live loop. It is off by default in the simulated-cell launch and set to every 10 s in `fabric_cell`. Decide after watching it in a shadow session.

## 5. Running it

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

## 6. What is left for you, in order

1. ~~**On this laptop (needs `sudo`):** install `ros-jazzy-rmw-cyclonedds-cpp` and `chrony`; attach a USB Ethernet adapter.~~ Done 2026-11-16; `scripts/hw/laptop_env.sh` writes the CycloneDDS profile (§8.1).
2. ~~**On the helpers:** build `oarbot_runtime` and `tablet_gui`.~~ Done 2026-11-16, in a separate `~/fabric_ws` on Blue, Silver and the tablet (§8.1); the NUC on 2026-11-17 (overhead camera only).
3. **Fill the `fabric_cell` templates:** sheet size and mass, tag size and tag-to-pinch offset, colour thresholds, `calibration.yaml` (the launch refuses to start while it says `measured: false`), workspace box and force limit in `bridge.yaml`, obstacle in `twin.yaml`.
4. **Hardware gates** from the plan (§5): carbon depth first, then H1–H3, H5, H7, H9, H12, H15. H3 translation passed on both robots on 2026-11-16 and confirmed the command frame in `bridge.yaml` (§8.2), and again on Blue on 2026-11-18 after its arm came back; H3 rotation is still open. H9: every host within 3.2 ms of the laptop on 2026-11-18 (tablet 7.3 ms).
5. **Shadow sessions before any closed loop:** estimator alone, then twin with `commands:=false`, then warm-up, then closed loop without an obstacle.

## 7. Changes outside this repository

`/media/taha/ExtermeLinux/oarbot_simulation/scripts/run_oarbot_fabric.py` gained `--external-followers` and `--external-timeout` (the plant's robots take grip velocities from `/oarbot_{blue,silver}/fabric_cbf/cmd_world`). That checkout already had uncommitted changes of yours; mine are additive. Rehearsal outputs are under `logs/sim_cell/hwprep/` there (about 2 GB).

## 8. Lab sessions on the shared robots

Other people use the same robots, so fabric sessions run beside the lab's own system without
changing it. `scripts/hw/README.md` is the runbook for every command below.

### 8.1 Network and isolation

- **Network.** This laptop's USB Ethernet (`enx3c8cf8eb0ec5`) is on the switch bridged to the
  `CII8218-oarbots` router. Its saved profile is `192.168.131.100`; the robots are on `192.168.1.0/24`,
  so add `192.168.1.200` after each boot or replug:
  `nmcli device modify enx3c8cf8eb0ec5 +ipv4.addresses 192.168.1.200/24`. The helpers are wired too
  (DHCP, Blue `.71`, Silver `.110`) and keep their Wi-Fi addresses `.102`/`.104`.
- **Isolation.** Each machine has a separate clone, `~/fabric_ws`, built by the lab account without
  sudo. A session stops the stock helper service (never disables it) and runs the fabric stack as
  transient user services on ROS domain 17 with CycloneDDS bound to the lab address; the stock system
  uses domain 0. A reboot always returns a machine to the stock system. `lab.sh snapshot` records the
  stock state; after every session so far it reported "unchanged"
  (`experiments/hardware/stock_snapshots/`).
- **The stock deploy service** (`*_cd.service`) pulls upstream `main` into `~/ros2_ws` and rebuilds at
  every boot. Keep fabric work on the fork.
- **Clocks.** Since 2026-11-17 this laptop is the cell's time server (chrony, campus server upstream);
  every other host's timesyncd asks it first. On 2026-11-18 all hosts were within 3.2 ms of the laptop,
  the tablet within 7.3 ms (`scripts/hw/clock_offsets.py`).
- **Blue's wrist sensor is plugged into Silver's laptop.** On Blue's own laptop it forms a ground loop
  with Blue's arm USB, and Blue's arm jitters and reports stale joint angles while moving (§8.4). Silver's
  session reads it (`hosted_ft:=blue`); the runbook has the details.

### 8.2 Gate H3, translation (2026-11-16)

`scripts/hw/first_motion.py` moved each arm from Home along each Kinova command axis and back,
through the guard and the tablet deadman. Results and 50 Hz samples:
`experiments/hardware/sessions/2026-11-16_h3_translation/`.

- **Command frame.** Kinova Cartesian x/y/z equals each arm's `j2n6s300_link_base` x/y/z (0.99 or
  better along the axis). Silver's base is turned 180 degrees on its chassis, so its x/y point the
  other way in the robot frame. `command_frame` in `bridge.yaml` is right.
- **Velocity response.** No motion at 1 cm/s and almost none at 2 cm/s; at 3-4 cm/s the arm delivers
  65-75 % of the command after 0.3-0.6 s. The twin's slow corrections fall in that deadband, and its
  actuator model should include the lag before closed loop.
- **Blue again, 2026-11-18** (`experiments/hardware/sessions/2026-11-18_blue_h3/`), after its arm came
  back from someone's firmware work and with its wrist sensor on Silver's laptop: same directions,
  clean joint readings, return error 2-4 mm. At 3 cm/s z delivered 3.8-3.9 cm of 5 (day 1: 2.8-2.9),
  x 2.2-2.4 cm at 1.6-1.7 cm/s (day 1: 3.0-3.1 cm at 2.0 cm/s), y 2.6 cm. Use today's numbers for
  Blue in the twin's actuator model.

### 8.3 Real robots executing the simulated cell's twin

The simulated cell stands in for the sheet and the people; the real arms execute the same safe
commands as the simulated ones:

```text
glove keys -> Isaac plant (oarbot_simulation) -> estimator -> fabric_twin + CBF + stretch guard
           -> /oarbot_{blue,silver}/fabric_cbf/cmd_world -> simulated robots (--external-followers)
                                                  \-> cmd_relay.py -> each helper's fabric_cbf_bridge
                                                     -> command_guard (tablet deadman) -> Kinova arm
```

- The simulation runs on domain 42 with Fast DDS and uses the same names as the robots, so only the
  two `cmd_world` topics cross, through `cmd_relay.py`. It restamps them with wall time (the twin
  stamps simulated time) and scales them by the real-time factor (about 0.46), so a real arm covers
  the same distance per second as its simulated copy.
- Each bridge uses the sim layout (`calibration_exact.yaml` of `fabric_sim_cell`) as its world frame.
  This makes a world-frame command move the real arm the way it moves the simulated one, relative to
  its own base; it is not a calibration of the lab. The real arms start from Home and the simulated
  ones from their grasp pose, so the motions match, not the positions.
- Each bridge keeps its tool in a box around Home, measured with MoveIt (`BOXES` in
  `scripts/hw/sim_bridge.py`): 10 cm down, 8 cm up, 12 cm forward and inward, 5 cm back, 4 cm outward.
  The outward side is short because the forearm and wrist camera approach the UWB wing. At a face of
  the box the bridge holds the motion that would go further out and keeps following the rest; it
  stops (latched) only 2 cm beyond the box. The force stop is 15.7 N (payload rating) on the
  0.25 s mean, so start-up jolts do not trip it. A bridge uses it only when its robot's calibrated
  force is published: on 2026-11-16 Blue had no sensor and ran without it; since 2026-11-18 Blue's
  sensor publishes from Silver's laptop.

Procedure, from the repository root on this laptop (each `lab.sh` call asks for the lab password):

1. Arms at Home (each arm's joystick home button, with no driver running).
2. `scripts/hw/lab.sh stock blue stop`, the same for silver; then `lab.sh session blue start`, the same
   for silver, and `lab.sh session tablet start` (close the stock tablet GUI first).
3. `lab.sh session blue start-bridge` and the same for silver.
4. Make sure both drivers accept commands; each answers `start_result='Arm started'`:
   `lab.sh session blue exec ros2 service call /oarbot_blue/kinova/j2n6s300_driver/in/start kinova_msgs/srv/Start`,
   the same for silver.
5. Terminal 1: `scripts/hw/run_sim_twin.sh 600 oracle`. Isaac Sim and RViz open in about 30 s; the
   simulated robots grasp about 20 s later.
6. Terminal 2, the glove keys (keep it focused): the command `run_sim_twin.sh` prints, with
   `--speed 0.1` so the real commands clear the deadband. Left glove W/S, D/A, E/Q; right glove
   I/K, L/J, O/U; Space stops both.
7. Terminal 3, after the grasp: `source scripts/hw/laptop_env.sh`, then
   `ros2 topic pub --once /oarbot_blue/fabric_cbf/arm std_msgs/msg/Bool "{data: true}"` and the same
   for silver. Each bridge's diagnostics then say "Following the fabric twin".
8. Tablet: Fabric Follow (CBF) on for both robots, Arm off; release the deadman once, then hold it.

If the real arms stop following: call the driver start service again (step 4), and check
`ros2 topic echo --once /oarbot_<robot>/diagnostics` for the bridge's message. After "tool left the
workspace box", home that arm and repeat steps 3 and 7 for it; after any other stop, repeat step 7.

To finish: Ctrl-C in terminals 1 and 2; `lab.sh session <robot> stop` for blue, silver and tablet;
home both arms; `lab.sh stock blue start` and the same for silver; `lab.sh snapshot` for each machine.

### 8.4 Problems found in the lab

| Problem | Effect | Status |
| --- | --- | --- |
| The patched Kinova driver stops the arm API after any gap over 0.2 s in its 100 Hz command stream, and only logs it | On 2026-11-16 both arms ignored commands for about 20 minutes while guard and bridge reported normally (Blue's helper froze at 21:31; Silver's cause unknown, 21:42) | Workaround: the start service (§8.3 step 4). To do: report the driver state in the guard diagnostics and the tablet GUI |
| Blue's helper laptop dropped off the network twice (both interfaces at once, no reboot) | Blue's guard and driver stall | To investigate: power and suspend settings |
| Calibrated wrist force runs at about 82-83 Hz with gaps up to 70-80 ms and ~37 ms age (raw 97 Hz), on both robots (2026-11-18) | The compensator waits for TF at each sample's exact time and drops samples older than 0.1 s | Not a blocker (the bridge's force timeout is 0.25 s); fix with a lag tolerance derived from the gravity error |
| Raw wrist force swings +/-4 N (peak 10 N) when the arm jerks into motion | An unfiltered contact threshold trips on vibration | Filter before thresholding |
| The plant's joint targets are not wrapped (`oarbot/robot.py`, `oarbot_simulation`) | After long runs PhysX rejects targets beyond +/-2 pi and a simulated arm stops following | To fix in the plant |
| Blue's Dingo computer did not boot on 2026-11-16 (panel: power blue, battery red, rest off) | No Blue base data that day | Boots again since 2026-11-17 (both bases parked by the overhead camera) |
| Blue's arm jitters and its joint readings go stale while it moves whenever Blue's Rokubi and Blue's arm USB are on the same laptop (2026-11-18) | A test run read 2 joint updates in 98 steps, then 80 mrad jumps, and aborted; the guard's workspace checks would be blind | Ground loop through the wrist-mounted sensor (charger and hub port make no difference). Workaround: Blue's sensor on Silver's laptop (`hosted_ft:=blue`). Fix: insulate Blue's sensor mount or fit a USB isolator |
| The Rokubi node stays up but silent after its sensor is replugged; the arm driver exits when the arm's USB is unplugged | Missing force or joint states with no session error | Restart that robot's session after any replug |
| Blue's stock Rokubi node exits at every start (no `/dev/sensor_bota_rokubi_ft_0` alias on Blue's laptop) | Stock admittance on Blue has no force data | Tell the lab; our session uses the `/dev/serial/by-id` path |
| The bridge latched a stop at the box face and on single force samples (first twin run, 22:22) | Blue stopped after 12.9 cm inward, Silver after 1 s on a start-up force spike | Fixed: soft box edge with a 2 cm hard margin, force stop on the 0.25 s mean |
| Silver's 1080p colour frames (about 8 MB) did not reach local subscribers with default DDS buffers | Wrist perception would get no images | The session profile asks for 64 MB receive buffers; frames arrive at 15 Hz |
