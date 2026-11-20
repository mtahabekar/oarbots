# Fabric hardware sessions

These scripts run the fabric cell on the shared OARBots without changing the stock system that
other people use. Run them from the compute laptop, from the repository root.

## Test day: start here

The procedure as run on 2026-11-19 (lab log `experiments/hardware/sessions/2026-11-19_lab_log.md`). Every step
is a script; nothing needs typing into ROS by hand. The laptop speaks the steps that need the deadman
(`spd-say`), so the person at the robots does not need the screen.

**0. Physical** (people at the robots): power robots, helpers, NUC, tablet; laptop USB Ethernet to the switch;
Blue's Rokubi USB on Silver's riser hub; wrist camera cables with slack and strain relief (both wrist cameras
have dropped off USB during motion; joint 6 turns up to ~120 deg in the box); tablet: fabric GUI, Fabric Follow
(CBF) on for both robots, deadman released.

**1. Bring-up and checks**
```bash
scripts/hw/cell_up.sh                   # lab address, hosts, clocks, code, stock services stopped, sessions, streams,
                                        # camera ages, arms; asks for the lab password once (or LAB_PASSWORD=...)
scripts/hw/fabric_run.sh build-laptop   # only after changing laptop packages: builds with ~/venvs/fabric_ros
                                        # (plain colcon breaks the twin: "No module named 'warp'")
scripts/hw/fabric_run.sh deploy-helpers # only after changing robot-side code: helpers to the fork's branch head
```

**2. Perception** (each run lasts the given seconds; restart it when it has expired: `check` shows its age)
```bash
scripts/hw/fabric_run.sh perception 3600
```

The launcher stages the current cell profile and camera runner from this checkout on all three hosts.
The wrist sheet crop uses the world workspace at the depth capture time. Keep the wrist views running
for diagnostics. The hardware fusion and estimator profiles currently use overhead and Blue. Silver's
remounted view needs hand–eye validation before it can contribute world-frame measurements again.

#### Calibrate the wrist cameras with the glove tags

Use this after adjusting a wrist mount. The existing glove diagnostic processes the registered RGB-D
streams on each camera host. The calibration measures the centres of glove tags 20 and 21. It uses
the surveyed overhead and robot-base poses, plus the wrist's end-effector pose at each image stamp
from TF. Both wrists can use the same overhead capture when they see the same tag.

For paper tags, tape the print flat to a rigid board with the white margins visible. Measure
the outer black square on each tag and record the widths in millimetres. Start near the centre
of the work area on a stable surface about 60 cm above the floor. Use a fresh session directory
for the paper captures and append `--paper TAG20_MM TAG21_MM` to each capture command below.
Paper mode searches the full RGB frame for IDs 20 and 21, estimates their centres from the
registered depth plane, and checks their recovered widths against the ruler measurements.
The normal glove colour settings remain in use for live hand tracking.

```bash
paper_session="experiments/hardware/sessions/$(date +%F)_wrist_calibration_paper"
scripts/hw/wrist_calibrate.sh capture both "$paper_session" fit_01 --paper 80 80
```

The example uses the cell's 80 mm tags. Keep the same widths for every hold in that session.
After collecting the paper fitting and check holds, use a saved empty capture from the same physical
camera mounts for the floor check. Run these commands from the repository root.

```bash
background="$(cat experiments/hardware/cell/obstacle_background.txt)"
scripts/hw/wrist_calibrate.sh fit silver "$paper_session" --floor-reference "$background"
scripts/hw/wrist_calibrate.sh fit blue "$paper_session" --floor-reference "$background"
```

The paper result also needs floor agreement within 2° and 3 cm at the search centre.
If tag centres alone leave a floor tilt error, add `--fit-floor <empty_capture>` to fit the floor
alongside the tag centres. Use a separate empty capture at a different arm pose for
`--floor-reference`. The calibration still has to pass the reserved paper placements.

For the measured-box check, record a fresh empty reference at the start pose, then place the
box in view of all three cameras. Substitute its actual outside dimensions in metres for
`WIDTH DEPTH HEIGHT`.

```bash
scripts/hw/fabric_run.sh obstacle background
scripts/hw/fabric_run.sh obstacle detect wrist_check --wrist-calibration "$paper_session" \
    --expected-size-m WIDTH DEPTH HEIGHT --check-box-registration
```

`--wrist-calibration` uses the qualified transforms for the captured point clouds. Use it for
matching captures made while the nominal wrist mounts were still in TF. Use a flat-topped upright
box for `--check-box-registration`. Each of the three views must recover the measured size from
the common top surface. Each wrist's top centre must be within 2 cm of overhead. The report also
records the full-cloud dimensions, which can be affected by foreground returns near the floor.
This check writes `box_registration.json` and creates no obstacle model. Omit
`--check-box-registration` to run the usual full-cloud size gate and model fit. Use `obstacle fit`
with the same options to repeat either check with saved depth.

```bash
session="experiments/hardware/sessions/$(date +%F)_wrist_calibration"
scripts/hw/wrist_calibrate.sh capture both "$session" fit_01
# Repeat with new fit_02, fit_03, ... labels, then use check_01, check_02, ... for held-out placements.
scripts/hw/wrist_calibrate.sh fit silver "$session"
scripts/hw/wrist_calibrate.sh fit blue "$session"
```

1. With the arms stationary and the deadman released, hold a glove tag flat and visible to overhead and
   the wrist being calibrated. Brace the glove on a stable surface if possible and keep it steady for
   the shared 8-second capture. The script reads sensors and TF. Arm positioning is a separate
   supervised operation using the [guarded joint-space procedure](#grasp-pose-and-silvers-fingers).
2. Collect at least five usable `fit_*` holds and three usable `check_*` holds per wrist. Spread the
   glove positions across width, depth and height. Use different positions for validation and include
   at least one different stationary arm pose, reached under the cell's supervised movement procedure
   with people, the sheet and the target support clear. Use `capture silver` or `capture blue` if the common
   view is too small.
   For paper, collect the fitting holds across at least two arm poses and keep a third pose for validation.
   Use several board placements at different heights and angles. A ground placement is useful as one check.
3. Each hold saves a log and timestamped tag and TF samples per camera. If a view reports too few pairs,
   inspect its log for tag visibility, depth or TF errors, then try a new label.
4. `fit` reports the residual per tag and hold and saves a candidate `right_mount` → `camera_base`
   transform in the session's `<robot>_result.json`. Qualification requires held-out p95 error at most
   2 cm and data from a different arm pose. The result is a candidate for review before changing the live
   TF. Check a measured obstacle before changing live TF.
   Rerecord the empty background after applying the transforms.

### 3. Capture and select a real obstacle

The same workflow handles one stationary, floor-supported object at a time. The detector fits a
convex footprint extruded from the floor to the top, so curved and irregular shapes are approximated
by that envelope. Clear the sheet and people from the camera views before capturing depth.

```bash
scripts/hw/fabric_run.sh start-pose --check
# If needed, return with start-pose and a tablet operator holding the deadman during the move.
scripts/hw/fabric_run.sh obstacle background  # optional if the saved reference still matches the empty cell
# Place the object under the sheet's intended centre and step out of view.
scripts/hw/fabric_run.sh obstacle detect new_object
# Review obstacle.png and fit.log at the printed path, then copy the printed use command.
scripts/hw/fabric_run.sh obstacle show
```

The empty reference in `experiments/hardware/cell/obstacle_background.txt` survives restarts and can
be reused when swapping objects. Refresh it after a change to the empty workspace, a camera mount,
or the camera calibration. Only a validated set of all three captures replaces the saved reference.
Both capture commands check the arm offsets before and after taking depth. They report when a guarded
return to the start pose is needed.

Each fit writes `obstacle.yaml`, a preview plot, OBJ and URDF files, the filtered point cloud, and
`fit.log` under `experiments/hardware/sessions/<date>_obstacle/`. The log records which views contributed,
why a view was skipped, and the reason for a failed fit. The twin sheet collides with the prism
(up to 18 footprint sides, with 1 cm of padding by default). The CBF uses the capsule that encloses it.
Check the pose and height against the physical object before running the printed
`scripts/hw/fabric_run.sh obstacle use <candidate_dir>` command. Selection takes effect at the
next `twin` or `go` launch. With no directory, `use` selects the last successful fit in this runtime.
Use the explicit directory to choose a particular scan or after a restart. Run `obstacle clear` to
restore the virtual box from `twin.yaml`.

#### Check dimensions or refit a saved scan

```bash
# Reject an automatic fit whose dimensions disagree with the tape measure.
scripts/hw/fabric_run.sh obstacle detect new_object --expected-size-m 0.1778 0.127 0.60

# Set scan to the raw capture directory printed by detect.
scan=experiments/hardware/sessions/2026-11-19_obstacle/obstacle2_remounted_180724
scripts/hw/fabric_run.sh obstacle fit "$scan"

# For this upright rectangular object, fit the pose from its visible top and use measured dimensions.
scripts/hw/fabric_run.sh obstacle fit "$scan" --measured-box-m 0.1778 0.127 0.60

scripts/hw/fabric_run.sh obstacle detect --help
```

Both size options take width, depth and height in metres. `--expected-size-m` checks the automatic
fit before padding, with a per-axis tolerance of 3 cm or 10%, whichever is larger.
`--measured-box-m` is for an upright rectangular object with a fully visible top. It uses only
filtered overhead depth for centre and heading and rejects an incompatible visible top. Both modes
apply the same padding. For an irregular shape, use the automatic fit.

`fit` runs offline and writes a new candidate directory. New scans store the matching background
path and the capture-time camera calibration. A legacy scan can recover its background from
`obstacle.yaml`. If the reference moved or the legacy capture has no record, supply the matching
empty capture as `obstacle fit "$scan" --background <directory> [detector options]`. The detector
reports when an older reference uses the current cell calibration. Capture a fresh pair if that
calibration cannot place the older cameras reliably.

#### If a fit fails

- **Camera moved or floor disagrees.** Check the mount and start pose. The detector compares pose,
  intrinsics, and floor between captures. Wrist views must agree with the reference floor within
  2° and 3 cm at the search centre to contribute. After a mount change, record a fresh empty reference
  and scan the object again.
- **Too few supported points.** Clear the view and check the camera diagnostics. The detector rejects
  unstable pixels, isolated returns, weak trails and unsupported changes in background depth holes.
  The defaults use a 3 cm depth change and 2 cm voxels. For a small object with reliable depth,
  use `fit` to try a smaller `--min-step` or `--voxel` and compare the result with the object.
- **Multiple objects or a clipped object.** Clear the sheet and clutter, then recapture. Use
  `--roi XMIN XMAX YMIN YMAX ZMIN ZMAX` in world metres to isolate the object or extend the search
  bounds. Leave visible floor for the alignment check.

A failed capture or fit leaves the selected twin model in place. Use `obstacle show` to distinguish
the selected model from the latest successful candidate before launching the twin.

**4. The run**
```bash
scripts/hw/fabric_run.sh go
```
In order: restarts both arms' Kinova velocity API (clears the driver's silent latch); if either arm is more
than 1.5 deg off the start pose, says "release, then press and hold the deadman" and brings both back (MoveIt-
checked joint path), "arms at the start pose, release the deadman"; starts the bridges (disarmed) and the twin
(commands off, RViz); says "twin ready" when it follows the measured grips; at the **first deadman press** after
that, commands on and both bridges armed ("commands on"; the formation is captured then: hold the sheet as it
should stay). Releasing the deadman stops both arms ("stopped"), pressing resumes ("moving"). **Ctrl+C** in that
terminal (or closing it, or `fabric_run.sh stop-go` from another terminal): commands off, bridges disarmed, twin stopped, the run saved (large files to
`/home/taha/fabric_hw_data`) and reviewed (`<run>/review.txt`).

**5. Checks at any time** (read-only)
```bash
scripts/hw/fabric_run.sh check               # arms vs start pose, guards (joint cap), bridges (stops), perception, camera ages
scripts/hw/fabric_run.sh start-pose --check  # how far each arm is from the start pose
scripts/hw/fabric_run.sh review <run folder> # per arm: commanded vs delivered speed, box faces, held/stopped windows
scripts/hw/fabric_run.sh obstacle show       # which obstacle the twin will use
```

**Check a change in the Isaac simulated cell first** (needs the ExtermeLinux disk): `src/fabric_sim_cell/scripts/run_twin_check.sh both`
compares today's settings and obstacle model with the baseline on the true sheet and table
(`src/fabric_sim_cell/README.md`, "Twin check").

**Step by step instead of `go`** (screen visible): `start-pose`, `bridges start`, `twin` (own terminal),
`status` until ready, `enable` + deadman, `watch 60`, `disable`, `stop-twin`, `save <run>`, `review <run>`.
`recentre` moves to the start pose whatever the offset; `arm-start` restarts the Kinova API alone.

**Limits in use** (since 2026-11-19, `experiments/hardware/cell/`): no arm stops on the box or the wrist force
(the box is a wall, the force is reported only); the twin reports a virtual collision but keeps commanding;
fault stops only (stale data, invalid commands, missing grips) and the deadman. Speeds from the arm: 36 deg/s
per joint (guard and bridge), 15 cm/s grip (bridge, twin). Boxes x +/-15, y +/-12, z -33/+9 cm around each
grasp grip (MoveIt sweep, `workspace_sweep.py`). The CBF margin is 9.5 cm, based on 2 cm of twin error plus
0.5 s of measured arm lag at 15 cm/s. In the 0.6 m Isaac descent, the true sheet cleared the table by at least
6.25 cm with this margin. The earlier 4.5 cm setting produced contact. See the lab log for the rehearsal and
Box1 preflight.

`cell_up.sh` re-adds the laptop's lab address, checks every host, the clocks (5 ms target), the code
deployed on each machine, stops the stock services (with sudo, password over ssh stdin), starts the fabric
sessions, then checks every stream, each camera's frame age on its own host and where the arms are; each
line says OK, WARN or FAIL and what to do. The first closed-loop procedure (2026-11-18), its settings and
results: `experiments/hardware/sessions/2026-11-18_s5_closed_loop/README.md`. The lab passwords are asked, not
stored: `lab_ssh.sh` keeps them in `$XDG_RUNTIME_DIR/fabric_lab` for the login session.

## Known problems and their fixes

| Symptom | Cause | Fix |
| --- | --- | --- |
| An arm does not move although its bridge says "Following" | Kinova driver latch "Velocity stream timed out: API stopped" after a >0.2 s gap | `fabric_run.sh arm-start` (`go` and `start-pose` do it) |
| The twin dies at start: `No module named 'warp'` | laptop packages built with plain colcon | `fabric_run.sh build-laptop` |
| A wrist camera stops publishing (`check`: no images) | USB drop during motion (cable strain) | fix the cable; `lab.sh session <robot> stop` and `start`, then `perception` and `bridges start` |
| `go` refuses: "A compute launch is already running" | a twin left running | `fabric_run.sh stop-twin` (`go` now stops a leftover twin itself) |
| The overhead's frames are 0.4-0.6 s old | full-resolution images viewed on the laptop fill the NUC link | view only the preview; restart the NUC session |
| Laptop has no 192.168.1.200 | USB Ethernet replugged or laptop rebooted | `cell_up.sh` re-adds it |
| A run executed code that was not part of it (2026-11-19 16:13: a `go` ran part of `deploy-helpers` and rebuilt Silver) | `fabric_run.sh` edited while a run from it was going; bash reads scripts as it runs them | fixed: `fabric_run.sh`, `start_pose.sh`, `cell_up.sh` are one `{ ... }` block, read whole before running |
| Simulated-cell scripts: "Simulation not found"; full_sim: "Isaac Sim python.sh not found" | the ExtermeLinux disk (Isaac Sim, oarbot_simulation) is not connected; hardware runs do not need it | connect it (`cell_up.sh` reports it) |
| `start-pose` aborts: "already has a publisher" | the bridges publish on the arms' joint input | fixed: it stops the bridges before its preflight |

## What stays untouched

The stock system on each machine is `~/ros2_ws`, the services that start it at boot, the scripts in
`/usr/local/bin`, udev rules, network settings and system packages. None of these are edited.

- The fabric code lives in a separate clone, `~/fabric_ws`, built by the lab account without sudo.
- A session only **stops** the stock helper service; it never disables it. A reboot always returns a
  machine to the stock system.
- The session runs as transient user services (`fabric-stack`, `fabric-cell`) that exist only until
  they stop or the machine reboots.
- The session uses ROS domain 17. The stock system uses 0; the Dingos use 1 (Silver) and 2 (Blue).
  Other people's tools on domain 0 cannot see or command the arms during a session. The Dingo
  bridges forward to whichever domain the session runs in.
- `stock_snapshot.sh` records the stock state. `lab.sh snapshot` compares each new record with the
  first one, so you can show that a session left nothing behind.

Not covered by software: while a session runs, nobody else can use the robots, and anything you
plug in, unplug or attach physically has to be put back by hand.

The stock deploy service (`*_cd.service`) updates `~/ros2_ws` from upstream `main` at every boot and
installs the result. Keep fabric work on the fork; anything merged upstream reaches every robot at
its next reboot.

## One-time setup

```bash
scripts/hw/lab.sh snapshot blue      # baseline of the stock system (also silver, tablet)
scripts/hw/lab.sh setup blue         # clone and build ~/fabric_ws at jazzy-platform-hardening (also silver, tablet)
```

Re-run `setup` to deploy a newer commit, or pass a tag: `scripts/hw/lab.sh setup blue hw-session-01`.
Robots run committed code only; `setup` refuses to update a workspace with local edits.

## A session

```bash
scripts/hw/lab.sh stock blue stop               # asks for the lab sudo password
scripts/hw/lab.sh session blue start            # drivers, guard, Dingo bridge on domain 17
scripts/hw/lab.sh session blue start-cell /abs/path/to/cell_dir   # wrist perception and CBF bridge
source scripts/hw/laptop_env.sh                 # this laptop on domain 17
# ... run compute.launch.py, record, test ...
scripts/hw/lab.sh session blue stop
scripts/hw/lab.sh stock blue start              # or reboot the laptop
scripts/hw/lab.sh snapshot blue                 # must report "unchanged"
```

The tablet has no stock service: close the stock GUI, then `lab.sh session tablet start` opens the
fabric GUI on the tablet screen. Only one GUI can hold the deadman Arduino at a time.

On the tablet itself, open the file manager, go to `fabric_ws/scripts/hw`, right-click
`tablet_gui.sh` and choose *Run as a Program*. Closing the GUI window ends it.

### Camera images stay on their hosts

**Never subscribe to a full-resolution camera image from the laptop** (RViz Image or Camera display,
`ros2 topic echo/hz`, a recorder). One 1080p BGRA stream at 15 Hz is about 110 MB/s and fills the camera
host's gigabit link. The camera driver's publish then blocks, captures back up in the Azure SDK, and the
frames stay 0.4-0.6 s old **until the camera is restarted**, even after the subscriber has gone. Perception
rejects frames older than 0.25 s, so that camera then publishes no sheet and no gloves. Measured
2026-11-18 21:40 on the overhead camera: 63 ms -> 564 ms while the laptop subscribed, 437 ms after; this
was why the overhead published nothing in the S2 overlay runs.

- The NUC session runs `preview_relay.py` (640 x 360, 5 Hz, `/nuc/preview/rgb/image_raw`); the cell's
  `fabric_cell.rviz` shows that, and its full-resolution wrist display is off.
- Before every recorded run, check each camera on its own host; the image age should be under 0.1 s:
  `lab.sh session nuc exec python3 fabric_ws/scripts/hw/frame_age.py /nuc 6` (and
  `/oarbot_blue/azure_kinect`, `/oarbot_silver/azure_kinect` on the helpers).
- A stale camera recovers only by restarting it: `lab.sh session nuc stop`, then `start` (a helper's
  session restart also restarts its arm driver: arms hold their pose, Silver's fingers stay closed).

## First arm motion (gate H3, translation)

`first_motion.py` moves one arm 1.5 cm along one Kinova command axis and back at 1 cm/s, with zero
rotation, through the command guard's `fabric_cbf` input. Before sending anything it checks that
the arm is at the SRDF Home state, that the guard is the only publisher to the driver, and that
MoveIt finds the 3 cm neighbourhood of the hand reachable without collision or configuration
change. It aborts on excess displacement, joint change, speed, stale data or (with a wrist sensor)
a force change that suggests contact.

1. Start the robot's stack and the tablet GUI. Do not start `fabric-cell`.
2. `lab.sh session silver exec python3 ~/fabric_ws/scripts/hw/first_motion.py --robot silver --check-only`
3. On the tablet, turn on **Fabric Follow (CBF)** for that robot only; leave Arm and Admittance off.
   Release and press the deadman once, then hold it. Releasing it stops the arm at any time.
4. Same command with `--axes z`, then `--axes xy` once z looks right. Reports and samples are saved
   under `~/fabric_ws/run/first_motion/` on the helper.

Per-robot defaults, such as unplugged devices and the F/T serial port, are in `hosts/<robot>.args`.
Arguments after `start` override them, for example `lab.sh session blue start camera:=true`.
`status` and `logs` show the session state and the last 200 log lines (`logs ft-blue` for a hosted sensor).

Paths passed through `exec` must not start with `~`: the laptop's shell expands it before SSH. Use
paths relative to the home folder, for example `exec python3 fabric_ws/scripts/hw/first_motion.py`.

### Grasp pose and Silver's fingers

The arms hold their sheet corners about 0.65 m ahead of their bases and 1.05 m high. Use
`grasp_target.py` on the laptop to check the simulated targets and camera views. For the physical
approach from Home, use the guarded joint-space procedure, with the sheet clear and one arm at a
time. Use the preflight to check each path with the other arm in its actual pose.

```bash
python3 scripts/hw/grasp_target.py --robot blue  # laptop, session env
scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot blue --check-only
```

With a clear cell and the operator at the tablet, run the same joint-space command without
`--check-only`. Repeat the preflight on Silver after Blue arrives. The guard requires Fabric
Follow and the deadman. After the arms are stationary, attach the sheet at the recorded binder
clips. Blue's fingers are electrically uncommanded, and Silver's guard requests closed fingers
while Fabric Follow is selected and the deadman is held. Check finger position after driver
startup. The driver calls `InitFingers` and Silver's fingers stayed closed on the observed restarts.

The short Cartesian inset using `grasp_drive.py` is deferred. The driver input received nonzero
commands at grasp with no measured displacement. See the [motion record](../../experiments/hardware/sessions/2026-11-18_sheet_capture/README.md#cartesian-attempt-and-final-state)
and [motion qualification plan](../../docs/fabric-hardware-test-guide.md#blocks-c-and-d-qualify-force-stops-and-the-grasp-pose-cartesian-envelope).

### Blue's wrist sensor is plugged into Silver's laptop

With Blue's Rokubi on Blue's own laptop, Blue's arm jitters and reports stale joint angles while it
moves: the sensor body is bolted to the wrist, and its USB and the arm's USB on one laptop form a
ground loop (lab log 2026-11-18, `experiments/hardware/sessions/2026-11-18_blue_h3/`). Until Blue's
mount is insulated or a USB isolator is fitted, Blue's sensor cable runs to Silver's riser hub:

- `hosts/blue.args` has `ft:=false`, so Blue's session starts no F/T node of its own.
- `hosts/silver.args` has `hosted_ft:=blue`. `lab.sh session silver start` then also starts Blue's
  node on Silver's laptop as `fabric-ft-blue` (`hosted_ft.launch.py`), with Blue's name, calibration and
  serial port (from `hosts/blue.args`). `session silver stop` stops it with the rest.
- Everything that uses Blue's force subscribes by topic, and Blue's calibrated force takes Blue's TF
  from the network, so nothing else changes. `hosted_ft:=none` after `start` leaves it out.
- Leave slack in the cable for Blue's arm motion and do not drive the bases while it is plugged in.

To move the sensor back: delete both lines, plug it into Blue's laptop, restart both sessions.

### Glove tags: check and calibrate a camera

`glove_check.py` runs on the computer a camera is plugged into (1080p frames do not cross the network)
and shows what the glove detector sees with that camera's perception settings, frame by frame: every
tag in the image, detected or not and why (colour support, search region, pose), the colour around it,
and the grip position (world frame for the overhead camera). It saves raw frames for offline tuning:

```bash
scripts/hw/lab.sh session nuc exec python3 fabric_ws/scripts/hw/glove_check.py --camera overhead --seconds 60
scripts/hw/lab.sh session blue exec python3 fabric_ws/scripts/hw/glove_check.py --camera blue_wrist --seconds 60
```

Output: `~/fabric_ws/run/glove_check/<camera>_<time>/` (`frames.jsonl`, saved colour and depth frames,
annotated JPEGs). `--params FILE` tries other settings live. Offline on the laptop, `glove_tune.py`
measures the glove colour near the tags, the look-alikes in the scene, and evaluates any parameter set
on the saved frames with annotated images. The calibration of all three cameras on 2026-11-18, step by
step: `experiments/hardware/sessions/2026-11-18_glove_test/README.md`.

To check a camera's real perception node live: `lab.sh session nuc start-cell <cell_dir>` on the NUC (on a
helper, `start-cell` also starts the CBF bridge; for perception alone run `fabric_cell
camera_host.launch.py camera:=<robot>_wrist` as a `fabric-cell` unit), then
`python3 scripts/hw/glove_record.py --camera <camera> --seconds 90 --out live.jsonl` on the session domain:
rate, age at arrival, per-hand detection, gaps and positions. The nodes keep up at 15 fps only with
`fabric_rate_hz: 5` (sheet processing at 5 Hz, gloves on every frame); a node that falls behind drops
frames for age and publishes no gloves at all. Laptop discovery of a newly started node can take a minute:
run the recorder on the camera's own computer with `exec` if it sees nothing.

For the next stationary stand-in trial, start the existing perception-only `camera_host.launch.py`
with `perception_overrides:=<cell_dir>/perception_workspace.yaml`. Use the measured overhead TF
from the session's calibration publisher when testing the world box. Record baseline and candidate
clips separately so the two nodes do not publish on the same camera topics. The
[labelled workspace trial](../../experiments/hardware/sessions/2026-11-18_sheet_capture/README.md#labelled-workspace-trial)
has the launch and replay details.

### After unplugging anything

- **F/T sensor:** the Rokubi node does not reopen its port after a replug. It stays up and silent,
  and `first_motion.py` then aborts waiting for force. Restart that robot's session.
- **Arm USB:** the arm driver exits (`KinovaCommException`); the guard keeps running without it.
  Restart that robot's session, then call the driver start service (below) before any motion.
- **Wrist camera USB:** Blue's camera dropped during the approach. Verify cable slack and
  confirm image and camera info streams after the session restart.

## Versions

- `sim-eval-2026-11-16` (in this repository and both simulation repositories) marks the code behind
  the simulation evaluation.
- Hardware and simulation work share one branch, `jazzy-platform-hardening`, so both always run the
  same algorithm code. Hardware-only files (cell measurements, these scripts, session records) stay
  in their own directories: `scripts/hw`, `experiments/hardware` and the cell configuration.
- Tag the commit each session runs (`hw-session-NN`). `~/fabric_ws/DEPLOYED` and
  `~/fabric_ws/run/sessions.log` on each machine record what ran and when.

## Real robots executing the simulated cell's twin

The simulated cell (`oarbot_simulation` plant, this repository's estimator, and `fabric_twin` with the
sensitivity CBF from `fabric_sim_isaacsim_newton`) runs on this laptop on domain 42. Its twin's safe
follower velocities drive the simulated robots (`--external-followers`) and, through `cmd_relay.py`
and each robot's bridge, guard and the tablet deadman, the real robots too. You jog the gloves.

1. Arms at Home; the session stack running on both helpers and the tablet GUI open.
2. `lab.sh session blue start-bridge` and `lab.sh session silver start-bridge`. Each checks Home and a
   per-robot workspace box around the Home tool position (`BOXES` in `sim_bridge.py`, measured with MoveIt) again, then starts the
   bridge with the sim layout as its world frame (a mirror for directions, not a lab calibration).
   A bridge uses the force stop only when that robot's wrist force is published; since 2026-11-18
   Blue's is (from Silver's laptop, see above).
3. `scripts/hw/run_sim_twin.sh [SIM_SECONDS] [oracle|gloves]` opens Isaac Sim and RViz and starts the
   relay; it prints the glove-key command for a second terminal.
4. After the robots grasp in the sim: `source scripts/hw/laptop_env.sh`, then arm each bridge with
   `ros2 topic pub --once /oarbot_<robot>/fabric_cbf/arm std_msgs/Bool "{data: true}"`.
5. On the tablet, Fabric Follow on for both robots (Arm off), deadman held. Jog the gloves.

Before arming, make sure each arm driver accepts commands (needed after any pause in the command
stream, see below); it answers `start_result='Arm started'`:
`lab.sh session <robot> exec ros2 service call /oarbot_<robot>/kinova/j2n6s300_driver/in/start kinova_msgs/srv/Start`.
To re-run `start-bridge` (for example after re-homing), stop the old bridge first with
`lab.sh session <robot> exec systemctl --user stop fabric-bridge`.

The patched Kinova driver stops the arm API when its 100 Hz command stream pauses for more than 0.2 s
(the helper froze, or heavy CPU load) and stays stopped until the start service is called. It only
logs this (`Velocity stream timed out`); the guard and bridge keep reporting normally. If the arms stop
responding while commands flow, call the start service. Do not rebuild on the helpers while the arms
are live.

The relay scales velocities by the simulation's real-time factor (~0.45), so the real arms move as
far per second as their simulated copies. The arms ignore commands below ~2.5 cm/s, so jog with
`--speed 0.1`. A bridge stop (workspace box, force, stale data) latches until you arm it again.
