# S2 estimator shadow with nominal grip geometry, 18 November 2026

The compute laptop ran the real-data estimator at the stationary grasp pose. Both robot grip
frames used the zero translation and identity rotation from the exact simulation calibration,
as requested. The existing glove settings already contained the simulation's tag-to-pinch
vector of `[0, -60, -15.5]` mm for each hand. The surveyed real-cell base and overhead camera
transforms supplied the remaining geometry. [Assumptions and provenance](assumptions.json)

The launch used `nominal_grips:=true twin:=false bank:=false commands:=false`. The flag works only
in estimator-only shadow mode. The camera hosts ran the same workspace profile used in the
stationary timing trial. The bag contains compact cloth and glove observations, TF, joint states,
estimator output and diagnostics. The camera nodes ran for 58 seconds starting at 19:18:49 EST.
The held timing window begins two seconds after the steady cue and ends at the last capture
common to the three camera streams. [Phase cues](live/phase_markers.txt)

## Measured result

| Metric | Result |
| --- | ---: |
| All saved state outputs, including predictions | 609 |
| Actual analyses in the 45.98 s held window | 436 |
| Analysis rate | 9.48 Hz |
| Capture stamp to estimator recorder, p50 / p95 | 212 / 273 ms |
| Longest interval between accepted analyses | 0.370 s |
| Longest interval between messages with both fused hands | 1.867 s |
| Analyses flagged degraded in the held window | 436 of 436 |

The run exercised camera-to-estimator transport and real-data analysis. The estimator marks
analysis over the 100 ms research target as degraded. Individual diagnostics also reported
missing fresh hand inputs and edge strain. The assumed grip geometry and the timing and hand
gaps limit what this run can establish about shape accuracy. All three cameras contributed
to the estimate, and this session has no independent material-marker reference.
[Timing and diagnostics summary](timing_metrics.json)

Each camera reported 4.7–4.95 cloth observations per second. Camera-side accepted publication
p95 ages were 154 ms overhead, 178 ms on Blue, and 165 ms on Silver. The postflight reported
disabled arm modes, released deadman, zero Cartesian commands, and unchanged joint readings.
[Camera logs and postflight](live/cameras/)

## Recordings

- [Compact bag metadata](live/compact_bag/metadata.yaml), estimate NPZ, glove states, and
  diagnostics are in [live](live/).
- The archive at
  `/media/taha/ExtermeLinux/claude-1000/lab/captures/2026-11-18_s2_nominal/`
  contains the bag, estimates, logs and configuration snapshot. [SHA-256 manifest](recordings.sha256)
- Before advancing the test, review hand dropouts during the motion clip and compare the
  estimator against a withheld camera or independent cloth markers. Keep the nominal grip
  assumption in the analysis record.
