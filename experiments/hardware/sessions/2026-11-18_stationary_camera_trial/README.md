# Stationary three-camera trial, 18 November 2026

Both arms held the grasp pose while the operator held the free corners steady, changed the
sag, and moved the corners slowly. A second recording started after the operator detached the
sheet. All three camera drivers were connected over Ethernet. The clock offsets before the
test were +2.4 ms on Blue, +2.3 ms on Silver, and +3.8 ms on the NUC relative to the laptop.
[Clock check](diagnostics/clock_offsets.txt)

## Recordings

| Phase | Start (EST) | Raw / live node duration | Overhead | Blue wrist | Silver wrist |
| --- | --- | --- | ---: | ---: | ---: |
| Held sheet | 18:39:26 | 52 s / 52 s | 49 pairs | 49 pairs | 49 pairs |
| Sheet detached | 18:42:05 | 15 s / 24 s | 14 pairs | 14 pairs | 15 pairs |

Each pair includes lossless RGB and registered-depth frames, plus the diagnostic JPEG. The
190 pairs, camera info, phase recordings and camera logs are archived at
`/media/taha/ExtermeLinux/claude-1000/lab/captures/2026-11-18_stationary_camera_trial/`.
The original files are also under `~/fabric_ws/run/stationary_camera_trial/` on the camera
hosts. All six transfers were verified against the [host checksums](manifests/). The
[phase markers](diagnostics/phase_markers.txt) log when the operator was cued and confirmed
the sheet was detached. Camera-local RGB/depth pairing is approximate. The runs do not contain
a complete joint, TF or force recording for the estimator.

The [compact recordings](live/) preserve the original frame and observation stamps, camera
intrinsics, raw capture parameters, glove detections and live diagnostics in the repository.
The candidate ran from a temporary source overlay from `f129a3f`. For the overhead world box,
the [measurement script](measure_live.py) placed the measured overhead transform in the
candidate node's own TF buffer, where it combined with the driver's factory transforms.

## Baseline versus candidate on the same frames

Three held frames per camera were annotated in 480 × 270 RGB, scaled to native resolution, and
scored with a 12 px outline tolerance. The frames sample the steady hold and two points during
slow motion. Another three frames per camera were scored after detachment. Some detached frames
include people outside the central work area. Everything in those frames is background for the
sheet detector.

| Camera | Candidate precision, held | Candidate recall, held | False cloth pixels per detached frame, baseline → candidate |
| --- | ---: | ---: | ---: |
| Overhead world box | 83.1–97.7% | 91.8–98.1% | 147,604–149,360 → 623–674 |
| Blue optical-depth band | 95.6–99.5% | 85.0–86.9% | 425,302–428,539 → 0 |
| Silver optical-depth band | 96.1–98.7% | 89.2–91.6% | 410,179–423,460 → 0 |

The baseline overhead mask recovered at most 0.34% of labelled cloth on these frames. The depth
bands removed the wrist background, and some visible cloth remained outside the registered-depth
field of view. The overhead candidate retained a small false mask near the attachments. That
mask still yielded 121–128 weighted boundary samples on the three detached frames, with 0–1
3-D point per frame. A background or gripper exclusion should be checked against both held and
detached frames before relying on those contours.

[Held comparison images](images/held_comparison.jpg) · [Detached comparison images](images/empty_comparison.jpg) ·
[Held scores](held_scores.json) · [Detached scores](empty_scores.json)

## Live publication timing

The settings target 5 fabric observations per second. Rates use the full process window.
Age is measured from the RGB capture stamp to publication on the camera host, and p95 includes
only accepted observations. The raw diagnostic recorder ran alongside the candidate for the
held test and for the first 15 seconds of the detached test.

| Camera | Held rate | Held age p95 | Held longest gap between observations | Detached rate | Detached age p95 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Overhead | 4.0 Hz | 189 ms | 0.20 s | 4.83 Hz | 243 ms |
| Blue wrist | 4.88 Hz | 171 ms | 1.00 s | 5.0 Hz | 155 ms |
| Silver wrist | 1.12 Hz | 247 ms | 28.21 s | 5.0 Hz | 176 ms |

Silver's gap ran from 18:39:39.836 to 18:40:08.047 while its raw RGB-D diagnostics continued
at roughly 2 Hz. The overhead candidate stopped yielding accepted observations about 10 s
before the held raw capture ended. Both reported the 250 ms age limit during the held test.
After the raw recorder stopped in the detached phase, accepted p95 ages were 230 ms overhead,
137 ms on Blue, and 157 ms on Silver. The held and detached clips contain different workloads,
so this trial does not isolate the cause of the held timing failures.
[Timing metrics](live_metrics.json)

The read-only postflight found no remaining test processes. Both guards reported only zero
Cartesian commands, disabled modes, released deadman and unchanged joints over six seconds.
[Blue postflight](diagnostics/end_blue.txt) · [Silver postflight](diagnostics/end_silver.txt)

### Perception-only held-sheet repeat

The repeat started at 18:50:24 EST and ran for 52 seconds on all three camera hosts. The
operator reattached the stand-in and repeated the steady hold, sag and slow corner motion.
The candidate and compact logger matched the first trial. The raw RGB-D diagnostic recorder
was absent. The arm guards were idle before and after the run. The clocks were within 2.6 ms
of the laptop. [Phase cues](diagnostics/solo_phase_markers.txt) · [Clock check](diagnostics/solo_clock_offsets.txt)

| Camera | Fabric rate | Accepted publication age p95 | Longest gap between accepted captures |
| --- | ---: | ---: | ---: |
| Overhead | 4.94 Hz | 151 ms | 0.20 s |
| Blue wrist | 5.00 Hz | 194 ms | 0.20 s |
| Silver wrist | 4.54 Hz | 216 ms | 3.60 s |

Silver rose from 58 to 236 accepted observations and its longest gap fell from 28.21 to 3.60 s.
The remaining gap, 18:50:26.819–18:50:30.420, occurred before the steady cue. In equal windows
after each cue, Silver published 5.00 Hz steady and 4.89 Hz moving in the repeat, versus
0.25 Hz and 0 Hz with the raw recorder. The overhead rate and p95 improved. Blue reached
5 Hz, with a higher accepted age p95 than the original 171 ms. This comparison supports a
recorder workload effect, especially on Silver. The two runs used manually repeated poses and
movement, so the runs do not establish that the recorder caused every timing loss.

The [comparison metrics](perception_only_metrics.json) use a 12 s window beginning 2 s after
the steady cue and a 9 s window beginning 2 s after the movement cue. Whole-run rates use 52 s.
The [compact logs](live/perception_only/) were checked against the host [SHA-256 manifests](manifests/).
The external copy is under `perception_only_185024/` in the same trial archive.
[Blue postflight](diagnostics/solo_end_blue_wrist.txt) · [Silver postflight](diagnostics/solo_end_silver_wrist.txt)

Use the compact logger for the next held-sheet timing checks and review the remaining Silver
gap before qualifying the camera inputs for estimator shadow. Grip transforms and glove
offsets are tracked in the [hardware test guide](../../../../docs/fabric-hardware-test-guide.md#94-lab-blocks-and-demo-sessions).

### Overhead attachment exclusion

The offline [mask candidate](images/overhead_static_exclusion_candidate.png) contains 505 pixels
that the world-box segmentation labelled cloth in at least five of six detached training
frames. With the existing gripper-mask dilation, it removed every false cloth pixel and
weighted contour from eight other detached frames. Recall on the three annotated held frames
fell by 0.40–0.62 percentage points. Each held frame still yielded 256 accepted depth
samples. The candidate is specific to the recorded camera and parked-arm geometry.
[Per-frame exclusion scores](overhead_exclusion_scores.json)

## Reproduce the image scores

From the repository root in the Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
session=experiments/hardware/sessions/2026-11-18_stationary_camera_trial
archive=/media/taha/ExtermeLinux/claude-1000/lab/captures/2026-11-18_stationary_camera_trial
export PYTHONPATH="scripts/hw:src/fabric_perception:src/human_hand_tracking:${PYTHONPATH:-}"
for phase in held empty; do
  python3 experiments/hardware/sessions/2026-11-18_sheet_capture/score_capture.py \
    "$archive/$phase" --workspace-labels "$session/${phase}_labels.json"
done
```
