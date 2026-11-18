# Gate H3, translation, Blue again; Blue's wrist-sensor ground loop (2026-11-18)

Blue's arm works again (it did not start on 2026-11-17). These runs repeat Blue's day-1 translation
test with `scripts/hw/first_motion.py` (5 cm legs at 3 cm/s, guard `fabric_cbf` input, tablet deadman,
Fabric Follow), first with Blue's Rokubi on Blue's own laptop, then with it on Silver's laptop.
Each folder holds `report.json` and 50 Hz `samples.csv` (hand pose, joints, command, forwarded
distance, wrist force). Day 1: `../2026-11-16_h3_translation/`. Lab log: `../2026-11-18_lab_log.md`.

## Runs

| Run | Blue's F/T | Outcome |
| --- | --- | --- |
| `blue_20261118_101621`, `_102011` | on Blue's laptop, unplugged and replugged | Aborted before motion: the F/T node had stopped publishing after the replug |
| `blue_20261118_102120` | on Blue's laptop | Preflight only: Home within 0.045 rad, 40 IK targets |
| `blue_20261118_102137` | **on Blue's laptop** | **Aborted on the -z leg: "hand speed 15.5 cm/s"** from a stale-then-jumping joint reading (below) |
| `blue_20261118_104742` | unplugged | Aborted: 0.269 rad from Home after the jitter tests |
| `blue_20261118_104857`, `_104908` | unplugged | Preflight passed after homing; the run timed out (Fabric Follow was off) |
| `blue_20261118_113532` | **on Silver's laptop** | Preflight only: Home within 0.008 rad |
| `blue_20261118_113651` | **on Silver's laptop** | z legs completed |
| `blue_20261118_113824` | **on Silver's laptop** | x and y legs completed |

## With the sensor on Blue's laptop: stale joint readings

| Blue, z at 3 cm/s | Joint readings that changed while commanded | Largest single 20 ms step | Wrist force sd while commanded |
| --- | --- | --- | --- |
| Day 1, no sensor (`2026-11-16 .../blue_20261116_212225`) | 370 of 454 | 17.8 mrad | (no sensor) |
| Today, sensor on Blue's laptop (`102137`) | **2 of 98** | **79.9 mrad** | 5.8 N |
| Today, sensor on Silver's laptop (`113651`) | 132 of 152 | 16.7 mrad | 4.6 N |
| Silver, day 1, with its sensor (`211914`, `212017`) | 355 of 499, 268 of 305 | 17-21 mrad | 5.2-5.6 N |

In `102137` the guard forwarded the full 5 cm of the +z leg while the reported joints did not change,
although the wrist force swung as on a moving arm. The readings then caught up one joint at a time:
joint 1 by -10.5 mrad and joint 3 by +79.9 mrad in single 20 ms steps (a Jaco2 joint moves at most
about 12 mrad in 20 ms). After the run every joint read differently from its start (q2 -34, q3 +69,
q4 +39, q5 -32, q6 -91 mrad), so the arm had moved and its reports came late. The driver logged no
error and the kernel no USB fault: the arm's controller itself reported stale angles.

Cause, isolated by the user with the joystick (no driver involved): Blue's arm jitters only when both
its own USB and the Rokubi's USB are connected to the same laptop. Unplugging either cures it;
unplugging the laptop's charger or moving the arm's USB from the riser hub to the laptop does not.
The sensor body is bolted to the wrist, so the two cables tie the wrist and the arm base to one ground
and motor current can return outside the arm's own wiring. Silver has the same USB layout (both
devices on the riser hub) and does not jitter, so the difference is probably in how Blue's sensor
is mounted (not yet measured: continuity from the F/T plug shell to the arm's USB plug shell, both
robots). With Blue's sensor on Silver's laptop there is no second connection, and Blue moves cleanly.

## With the sensor on Silver's laptop: H3 translation passes on Blue

| Leg | Moved of 5 cm | Steady speed | Lag | Direction (arm base = robot base on Blue) |
| --- | --- | --- | --- | --- |
| +z / -z | 3.91 / 3.77 cm | 2.36 / 2.42 cm/s | 0.35 / 0.41 s | (0.00, -0.03, 1.00) / (0.00, 0.00, -1.00) |
| +x / -x | 2.24 / 2.44 cm | 1.63 / 1.71 cm/s | 0.56 / 0.49 s | (0.98, 0.08, -0.16) / (-0.99, -0.09, 0.12) |
| +y / -y | 2.65 / 2.62 cm | 2.00 / 1.97 cm/s | 0.49 / 0.48 s | (0.00, 1.00, 0.07) / (-0.02, -0.99, -0.17) |

Return error 1.9 mm (z run) and 4.0 mm (x/y run). Joint readings changed in 235 of 305 commanded steps
of the x/y run (largest step 14.3 mrad). The wrist force at the end of each leg was within 1.5 N of its
value before the leg, so the cable across to Silver did not pull on the wrist.

Against day 1 (no sensor; moved 2.78-3.35 cm at 1.83-2.19 cm/s on every axis), z now delivers more
(3.8-3.9 cm) and x less (2.2-2.4 cm at 1.6-1.7 cm/s). Blue's controller had its firmware worked on
between the two days; the twin's actuator model should use today's numbers for Blue. Directions match
day 1, including up to 17 % cross-motion on z during x and y legs.
