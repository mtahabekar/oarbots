# First closed-loop runs: the fabric twin driving both arms (2026-11-18, 22:35-23:36)

The real sheet (stand-in, binder clips on the robot corners), held by the person's gloved hands and by both
arms; three cameras feed the estimator; the twin with its CBF and stretch guard runs on the laptop and
commands both arms through the joint-mode bridges, guards and the tablet deadman held by a second person.
Robot grips use the simulation's nominal transforms (not yet measured, gate H15).

## Results

| Run | Outcome |
| --- | --- |
| [22:35 shadow](../2026-11-18_s3_shadow/) (commands off) | The twin ran 6 min but stopped on "a grip measurement is missing" ~90 % of the time: the overhead published nothing 22:33:50-22:46 (its camera driver re-based the camera clock when the NUC's time sync changed server; timestamps 540 ms off, all frames outside perception's 0.25 s window) |
| [23:14 closed loop](live_231053/) | Blue moved 7 cm toward the person and 6 cm up to its box face and tripped its 6 N force stop within seconds; Silver followed 13 cm down to its box face ("moving well", user). The formation command sat at its 10 cm/s cap 99-100 % of the time: offsets captured 1.5 min before enabling, and fused hands with impossible values (z -0.03 m, 95 cm steps). Fixed in `bc70581` |
| [23:30 closed loop](live_232055/) | "A smooth run" (user). About 80 s commanded; both arms followed until held at their box faces; Silver's force stop tripped near the end; Silver's wrist camera dropped off USB at 23:31:58 during the motion. Twin: stale grip 2 % of frames, error vs the estimate 21 cm p95, its sheet within 5 cm of the virtual obstacle. Formation command at its 10 cm/s cap 93-96 % with the bridges capped at 5 cm/s, so the cap was raised to 10 cm/s (`1da971a`) |

Hands seen by the twin while commanded (23:30): right hand 5.5 cm/s median (p95 26), left 6.5 (p95 30). The
right hand is the weak input: the overhead often misses its tag, and only Blue's wrist camera sees it (~7 %).

## How to reproduce, from a cold start

Everything runs from the compute laptop, repository root, branch `jazzy-platform-hardening`. Scripts:
`scripts/hw/cell_up.sh` (bring-up and checks), `scripts/hw/fabric_run.sh` (the run), `scripts/hw/twin_monitor.py`
(go/no-go readout). Background: `scripts/hw/README.md`, `docs/fabric-hardware-test-guide.md` §8-9.

**Physical setup (people at the robots)**
1. Power the robots, helpers, NUC and tablet; the arms come up at Home. Laptop USB Ethernet to the switch.
2. Blue's Rokubi USB on Silver's riser hub (ground loop, lab log 2026-11-18); wrist camera cables with slack at
   the wrist (both wrist cameras dropped off USB once during arm motion on 2026-11-18).
3. Sheet: 84 x 84 cm stand-in, a universal binder clip on each robot corner (8-9 g) to fix it on the fingers.
   Silver's fingers stay closed (its guard holds them, `hosts/silver.args hold_fingers:=100`); Blue's gripper is
   not driven.
4. Gloves: tag 21 on the left hand, tag 20 on the right, on the backs of the hands. The person stands in front
   of the robots; hands flat at waist height with the tags facing up (toward the overhead camera).
5. A second person at the tablet: fabric GUI, Fabric Follow (CBF) on for both robots, deadman released.

**Bring-up and checks**
```bash
cd ~/ros2_colcon_ws/src/oarbots
scripts/hw/cell_up.sh            # asks for the lab password once; --deploy to update every machine; --check to only look
```
It re-adds the laptop's 192.168.1.200, checks every host, clocks, deployed code, stops the stock services, starts
the fabric sessions (helpers, NUC with the overhead preview, tablet GUI), checks every stream, each camera's frame
age on its host and where the arms are. Fix any FAIL before going on (a silent camera: restart its session).

**Arms to the grasp pose** (first time from Home, or after a run left them elsewhere)
```bash
scripts/hw/fabric_run.sh recentre       # joint space to grasp.yaml, both arms; deadman held ~25 s from Home, ~5 s otherwise
```
From Kinova Home a Cartesian path crosses a wrist singularity, so this is a joint-space move (`grasp_joints.py`,
MoveIt-checked every degree, 15 cm from the other arm). At the grasp pose Kinova's Cartesian control is refused
(CartesianFaultState 1); the bridges therefore send joint velocity (`../2026-11-18_kinova_status/`,
`../2026-11-18_joint_response/`). Clip the sheet's robot corners on after the arms arrive.

**The run** (separate terminals)
```bash
scripts/hw/fabric_run.sh perception 3600   # T1: perception on all three cameras (workspace profile), 1 h
scripts/hw/fabric_run.sh bridges start     # T1: joint-mode bridges, disarmed; per-robot box, 10 cm/s, 6 N force stop
scripts/hw/fabric_run.sh twin              # T2: estimator + twin + CBF + RViz, commands OFF; prints its run folder
scripts/hw/fabric_run.sh check             # T3: arms, guards, bridges (stops, box), perception, camera ages
scripts/hw/fabric_run.sh status            # T3: repeat until: twin "following the measured grips", stale grip a few %,
                                           #     three cameras ~15 Hz, both hands detected, deadman released
scripts/hw/fabric_run.sh enable            # T3: commands on (formation captured now) + both bridges armed
#   tablet: release, press and hold the deadman; the person moves slowly (a few cm/s)
scripts/hw/fabric_run.sh watch 60          # T3: a line every 5 s (commands, bridges, deadman)
scripts/hw/fabric_run.sh disable           # T3: commands off + bridges disarmed
scripts/hw/fabric_run.sh stop-twin         # T3 (or Ctrl+C in T2): writes estimate.npz, twin.npz, twin.cbf.jsonl
scripts/hw/fabric_run.sh save <run folder> # large files to /home/taha/fabric_hw_data, camera logs from the hosts
scripts/hw/fabric_run.sh review <run folder> # per arm: commanded vs delivered, faces, where it stopped
```
Since 2026-11-19 (`8562767`, lab log 2026-11-19) the box is a wall only and there is no force stop; code updates
for the helpers: `fabric_run.sh deploy-helpers`. Stops (2026-11-18 settings): releasing the deadman stops both arms at once. A bridge stops itself (latched; `enable` re-arms) on a 6 N
change of its wrist force or 2 cm beyond its box. Only one compute launch may run (the 23:30 attempt first found
the 23:14 one still running: two twins; `fabric_run.sh twin` refuses to start a second).

## Settings used (`experiments/hardware/cell/`)

| File | What matters here |
| --- | --- |
| `bridge.yaml` | `output: joint`, `max_joint_dps: 15`, `max_speed_mps: 0.10` (5 cm/s in the 23:14 and 23:30 runs), `force_limit_n: 6` (placeholder), per-robot boxes: x +/-8 cm, y -6/+12 cm, z -12/+6 cm around the grasp grip, corners reachable in MoveIt |
| `twin.yaml` | CBF clearance only, 4 cm margin, 0.1 m/s; stretch guard 3 cm slack; formation slack 4 cm (2 cm lateral), gain 2/s; hand coast 1 s; virtual obstacle 44 x 44 x 32 cm box under the sheet's centre; density 0.215 kg/m^2 |
| `fusion.yaml` | hand workspace box x +/-0.9, y -0.35..1.2, z 0.4..1.6 m (since `bc70581`) |
| `perception_workspace.yaml` | sheet colour, overhead world box, wrist depth band (applied by `live_perception.sh`) |
| `calibration.yaml` | surveyed overhead and parked bases; grip transforms nominal (`measured: false`, so `nominal_grips:=true`) |

## Files

| Where | What |
| --- | --- |
| `live_231053/`, `live_232055/` (here) | `compute.log`, `twin.status.jsonl`, `twin.cbf.jsonl.gz` (every CBF update: hand and nominal velocities, commands, stretch guard), `estimate.status.json`, `estimate.hands.npy` |
| `/home/taha/fabric_hw_data/2026-11-18_closed_loop/` | `estimate.npz`, `twin.npz` per run (72-208 MB, not in git); checksums in its `MANIFEST.sha256` and below |
| `cameras/*_perception_logs.tar.gz` | each camera's perception output during the runs (gloves, observation ages) |
| `*_bridge_and_recentre.tar.gz` | each helper's bridge log and the last recentring moves |
| `../2026-11-18_s3_shadow/` | the 22:35 shadow run (same layout) |

Large-file checksums (SHA-256):

```
6864d4bc6a2d6306e2e7...  s3_shadow_223510/estimate.npz      55326325055f0963005d...  s5_231053/twin.npz
fc06d64f8f3bfcadcf46...  s3_shadow_223510/twin.npz          8269cf0d6ba68a9d3c5f...  s5_232055/estimate.npz
60b68bf8e0d502803c79...  s5_231053/estimate.npz             8aad8e2832d75e2255f3...  s5_232055/twin.npz
```
(full values in `/home/taha/fabric_hw_data/2026-11-18_closed_loop/MANIFEST.sha256`)

## Open after these runs

1. **Hand inputs:** the right hand's tag is often invisible to the overhead camera while pinching; fused hands
   still jump. Tags that stay facing up (or a second view of the person), measured tag-to-pinch offsets.
2. **Twin vs estimate:** 21 cm p95. Nominal grips, sparse hands and the twin's slack sheet all contribute.
3. **Wrist camera cables:** both dropped off USB once during arm motion. Strain relief at the wrist.
4. **Overhead timestamps after a time-sync change:** the driver's camera-clock offset can come out ~0.5 s wrong
   and take ~10 min to recover. Make perception or the driver robust to it.
5. **Force stop:** 6 N is a placeholder (H5: measure). Blue's compensated force reads ~39 N at rest.
6. **Grip transforms (H15)** and wider boxes, after checking a larger region in MoveIt.
