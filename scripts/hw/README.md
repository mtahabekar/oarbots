# Fabric hardware sessions

These scripts run the fabric cell on the shared OARBots without changing the stock system that
other people use. Run them from the compute laptop, from the repository root.

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
