# S3: twin and CBF in shadow on the real cell (2026-11-18, 22:35-22:43)

Started by the user with `compute.launch.py nominal_grips:=true twin:=true bank:=false commands:=false
rviz:=true record:=...` (commands off, bridges inactive) and perception on all three cameras
(`scripts/hw/live_perception.sh`). The twin started at 22:37:47, after waiting ~2.5 min for both hands and
both grips, and ran 6 min.

It stopped on "a grip measurement is missing" ~90 % of the time (685 of ~740 status rows; 46 "following"):
the overhead camera published nothing from 22:33:50 to 22:46. Its driver re-based the camera clock at
22:33:51 and 22:35:28 (the second when the NUC's timesyncd contacted another campus NTP server, jitter
503 ms); the offset came out 540 ms wrong, so every frame fell outside perception's 0.25 s window, and it
recovered over ~10 min. The wrists saw a hand in 13 % (Blue) and 3 % (Silver) of their frames. Twin frame
time 15-23 ms p95. Hand measurements reaching the estimator: 1.8 and 0.4 Hz, fresh within the twin's 1 s
window 22 % and 8 % of the time.

The closed-loop runs that followed: `../2026-11-18_s5_closed_loop/README.md` (procedure and results).

Files: `live/` (compute log, twin status and CBF records, estimate status and hands),
`cameras/*_perception_logs.tar.gz`; `estimate.npz` and `twin.npz` in
`/home/taha/fabric_hw_data/2026-11-18_closed_loop/s3_shadow_223510/` (checksums in its `MANIFEST.sha256`).
