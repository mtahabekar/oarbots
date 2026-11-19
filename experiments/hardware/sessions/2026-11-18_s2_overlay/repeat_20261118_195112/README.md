# S2 operator repeat. Mesh jumps and latency

The saved estimate spans 20:01:47–20:04:23 EST on 18 November. It matches the camera
capture stamps in [the 20:01 camera logs](camera_runs/rviz_repeat_20261118_200122/).
The [earlier camera logs](camera_runs/rviz_repeat_20261118_195130/) belong to the separate
19:51 perception run. Both rounds produced zero overhead observations and glove messages.
The overhead node reported rejected timestamps. The earlier workspace restart had restored
fresh frames during its short check.

| Metric | Measured result |
| --- | ---: |
| Saved states | 1,861 |
| Camera analyses / predictions | 897 / 964 |
| Analysis rate through last fused glove sample | 5.08 Hz |
| Capture to estimator recorder through last fused glove sample, p50 / p95 | 190 / 252 ms |
| Largest successive analysis-only node displacements | 65.7 and 63.5 cm |
| Display-stream transitions to an earlier state stamp | 115 |

The two largest jumps were at 20:02:12.612 and 20:02:51.815. The first followed a
4.74 s left-hand tracking gap. The second followed a gap of around 31 s in both fused
hand streams. The hand-held corners displaced the most at both events, while the
robot-held corners remained fixed to within the recorded numerical precision. The
analyses on either side of each jump used consecutive wrist observation intervals about
200 ms apart, so the gaps were in the hand measurements. The repeat has no operator cues
or matched overhead images, leaving the physical cause of each hand gap unverified.

With the configured 1 s coast window, the observer can continue to drive a human corner
from an extrapolated glove state before holding the corner. The returning hand measurement
then becomes a prescribed boundary target. This can cause a snap even while wrist depth
continues to arrive. The two largest jumps are present among analysis states alone. RViz adds
apparent jitter by drawing predicted and delayed analysis states as they arrive in the same
color, without synchronization to the camera image.

Across the full repeat, all 897 analysis stamps matched wrist camera logs. Median age was
103 ms on camera publication and another 53 ms from publication to the recorder. These are
stage medians over the whole repeat. The 190/252 ms figures in the table cover the earlier
window up to the last fused glove sample. The repeat has no independent reference for
quantifying surface error.

## Next stationary A/B check

1. Verify fresh RGB-D and glove observations at all three camera hosts under live load.
   Keep the reduced RViz preview. Track the NUC frame age over a longer window because
   the timestamp rejection recurred after the restart.
2. Compare capture-time analyses with predictions disabled and record the consumed
   grips and factors using `runtime.boundary_log`. Use the same static hold and short
   motion, then introduce a brief one-hand occlusion to measure reacquisition.
3. Evaluate a shorter hand coast and a covariance- and displacement-aware reacquisition
   gate on the same clip. Have the display indicate stale or missing hands and old mesh
   states. Compare time-aligned overlays for actual fit.
4. Once inputs are reliable, profile camera publication delay and the time from
   publication through assimilation. Check projected grip locations and fixed offsets
   on a steady frame before tuning material parameters.
