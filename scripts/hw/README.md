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
`status` and `logs` show the session state and the last 200 log lines.

## Versions

- `sim-eval-2026-11-16` (in this repository and both simulation repositories) marks the code behind
  the simulation evaluation.
- Hardware and simulation work share one branch, `jazzy-platform-hardening`, so both always run the
  same algorithm code. Hardware-only files (cell measurements, these scripts, session records) stay
  in their own directories: `scripts/hw`, `experiments/hardware` and the cell configuration.
- Tag the commit each session runs (`hw-session-NN`). `~/fabric_ws/DEPLOYED` and
  `~/fabric_ws/run/sessions.log` on each machine record what ran and when.
