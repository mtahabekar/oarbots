# Joint-velocity response at the grasp pose (2026-11-18, 22:17-22:24)

Both arms at the grasp pose, where Kinova reports CartesianFaultState 1 and ignores Cartesian velocity
(`../2026-11-18_kinova_status/`). `scripts/hw/joint_response.py` started the real `fabric_cbf_bridge`
in joint mode (`99c2572`, box +/-4 cm around the grip, 10 deg/s joint cap), armed it, and sent 2 cm
legs along each world axis and back at 1, 2 and 3 cm/s through the bridge's normal input; the user held
the deadman with Fabric Follow on. No abort on either arm.

| Robot | Commanded | Steady speed delivered | Lag to first motion | Travel of 2 cm | Direction (cosine) | Tool turn |
| --- | --- | --- | --- | --- | --- | --- |
| Blue | 1 cm/s | 84-95 % | 0.37-0.84 s | 1.63-1.86 cm | >= 0.986 | <= 0.18 deg |
| Blue | 2 cm/s | 88-97 % | 0.24-0.49 s | 1.67-2.02 cm | >= 0.972 | <= 0.37 deg |
| Blue | 3 cm/s | 83-93 % | 0.22-0.35 s | 1.78-2.07 cm | >= 0.980 | <= 0.40 deg |
| Silver | 1 cm/s | 84-94 % | 0.30-0.74 s | 1.58-2.00 cm | >= 0.994 | <= 0.10 deg |
| Silver | 2 cm/s | 85-96 % | 0.24-0.41 s | 1.71-1.93 cm | >= 0.988 | <= 0.35 deg |
| Silver | 3 cm/s | 86-99 % | 0.18-0.34 s | 1.82-2.09 cm | >= 0.990 | <= 0.40 deg |

Return to the start within 2.0 mm (Blue) and 1.4 mm (Silver). No deadband: Cartesian velocity near Home
moved nothing at 1 cm/s and almost nothing at 2 cm/s (`../2026-11-16_h3_translation/`). Blue's -x legs
carry 17-23 % sideways (the largest off-axis share of any leg; watch it). The shortfall of travel (0.1-0.4 cm of 2 cm)
is the lag: the leg's commanded time ends before the arm catches up.

Files: per robot `<robot>_<time>.json` (legs), `_samples.csv` (grip in the world, tool turn, forwarded
joint rates at 50 Hz), `_bridge.log`.
