# Warm-up design study, 2026-11-21 (simulation only, no robot motion)

Which warm-up motion lets the calibration tell bending, shear, stretch and the grip points apart? Produced by
`scripts/hw/fabric_run.sh warmup-design` (scripts/hw/warmup_design.py); method, results and conclusions in
`../2026-11-21_lab_log.md`.

| folder | candidates |
| --- | --- |
| robot_moves/ | current 2-4 cm insets; deeper insets; robots sideways, lift, narrow; combinations; the recorded carry live_115658 40-120 s as an anchor |
| person_moves/ | the person's hand moves on cues (15 cm), one at a time, all, and with the robot excursions |
| person_size_speed/ | the same hand moves larger (25 cm) or faster (30 cm/s): which matters |
| deployed/ | the profile in cell/twin.yaml since 2026-11-21 |

Each folder: result.json (every number; per candidate: material effects and their part no grip offset can imitate,
the compensating offsets, Cramér-Rao sd, geometry against the bridge workspace boxes and the start pair lengths),
report.md (the table), run.log. motion_stats.txt: how far and fast the corners moved in two carries and the old warm-up
(motion_stats.py, run with the offline environment of fabric_run.sh).
