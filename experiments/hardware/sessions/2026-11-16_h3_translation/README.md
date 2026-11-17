# Gate H3, translation: first supervised arm motion (2026-11-16)

Both arms started at the SRDF Home state. Commands went through `scripts/hw/first_motion.py`, the
command guard's `fabric_cbf` input and the tablet deadman (Fabric Follow selected), on ROS domain 17
with the stock services stopped. Each leg commands a constant velocity until the guard has
forwarded 5 cm, then holds zero for 1.5 s; "moved" is the measured hand displacement over the leg
and settle. Folders hold each run's `report.json` and `samples.csv` (50 Hz).

## Command frame

Kinova Cartesian velocity x/y/z is the arm's own `j2n6s300_link_base` x/y/z on both robots:

| Robot | Kinova +x, +y, +z in the arm base frame | In the robot base frame |
| --- | --- | --- |
| Silver | +x, +y, +z (0.999 or better along the axis) | -x, -y, +z (arm base turned 180 deg about z) |
| Blue | +x, +y, +z (0.989 to 0.998; up to 15 % cross-axis on -y) | +x, +y, +z |

This confirms `command_frame: <robot>/j2n6s300_link_base` in `src/fabric_cell/config/bridge.yaml`.
Rotation axes were not tested.

## Velocity response (Silver z sweep; both arms at 3 cm/s)

| Robot, axis | Command | Moved of 5 cm | Speed, second half of leg | Delay to 2 mm |
| --- | --- | --- | --- | --- |
| Silver z | 1 cm/s | 0.00 cm | 0 | none in 1.5 s |
| Silver z | 2 cm/s | 0.2 / 0.04 cm | 0.1 cm/s | 1.9 s |
| Silver z | 3 cm/s | 3.1 / 3.5 cm | 2.3 cm/s | 0.4-0.5 s |
| Silver z | 4 cm/s | 3.4 / 3.4 cm | 2.9-3.1 cm/s | 0.3-0.4 s |
| Silver x, y | 3 cm/s | 3.2-3.6 cm | 1.2-2.4 cm/s | 0.3-0.4 s |
| Blue z, x, y | 3 cm/s | 2.8-3.4 cm | 1.8-2.2 cm/s | 0.3-0.6 s |

The firmware ignores Cartesian velocity commands below roughly 2-2.5 cm/s and delivers about 65-75 %
of the command above it, after 0.3-0.6 s. The guard output reaching the driver was a steady 100 Hz
(jitter 0.07 ms), the QoS of guard and driver match, and the driver passes the velocity to
`SendAdvanceTrajectory` unscaled, so the deadband is in the arm. Small, slow follower corrections
from the fabric CBF would be lost; the CBF speed range and the twin's actuator model must account
for it before closed loop.

## Other observations

- The raw wrist force on Silver swings +/-4 N (one sample 10.4 N) when the arm jerks into motion.
  A contact check must filter it; the script now uses a 0.25 s mean and the 1.6 kg payload rating.
- Run `silver_20261116_211228` (1 cm/s) and the first leg of `silver_20261116_211542` produced no
  motion; `211542` was then stopped by the old unfiltered 5 N force check.
