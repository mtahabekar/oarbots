# Stand-in sheet at the grasp pose, 18 November 2026

The first three-camera stand-in capture used both arms at the simulation-derived grasp pose, with the
sheet clipped to the robot fingers and a person holding the other two corners. The last recorded arm
motion was the joint-space approach. The subsequent short inward Cartesian moves showed no measurable
travel and were cancelled. See the [grasp setup](../2026-11-18_lab_log.md#grasp-pose-block-a-begins-1640-1715)
and the [remaining test plan](../../../../docs/fabric-hardware-test-guide.md#9-the-rest-of-the-hardware-tests-plan-18-november-2026).

## Capture and artifacts

- At 17:21 EST, `glove_check.py` ran camera-local for 90 s at a requested diagnostic rate of 2 Hz, with
  one RGB/depth pair saved about every two seconds. The overhead perception node was stopped for the
  capture. Each view has 169 diagnostic rows and 43 saved 1920 × 1080 RGB/registered-depth pairs, with
  camera intrinsics and an annotated JPEG per pair. This is a sparse RGB-D recording. The capture
  has no time-aligned joint, TF or force streams for a full estimator replay.
- The [contact sheet](images/sheet_contact.jpg) shows frames 5, 22 and 38. Frame 5 shows the steady
  hold at ~11 s, frame 22 shows sag at ~47 s, and frame 38 shows the sheet hanging from the robot clips
  after the person released it at ~81 s. Frame 38 still contains cloth. An empty scene and a separate
  empty-clips/grippers sequence remain to be captured.
- The original RGB-D files are in `/home/taha/fabric_hw_data/2026-11-18_sheet_capture/`. A checksum-verified
  copy is in `/media/taha/ExtermeLinux/claude-1000/lab/captures/2026-11-18_sheet_capture/`.
  The [SHA-256 manifest](raw_files.sha256) covers all 399 capture files. The [glove session](../2026-11-18_glove_test/README.md)
  holds the earlier live recordings and its own raw archive entry.
- The [motion reports and compressed sample CSVs](motion/) come from the helpers' `run/grasp_joints`
  and `run/grasp_drive` directories. The [diagnostic logs](diagnostics/) preserve console output and
  the sensor/driver status check. The original stdout, reach/IK scans and exploratory files remain
  under `/media/taha/ExtermeLinux/claude-1000/lab/`.

## What the recordings show

| View | Tag 21, left hand | Tag 20, right hand | Both | Nonzero depth in probes, frames 5 / 22 |
| --- | ---: | ---: | ---: | ---: |
| Overhead | 112/169 | 88/169 | 87/169 | 100.0% / 92.3% |
| Blue wrist | 20/169 | 0/169 | 0/169 | 86.1% / 98.3% |
| Silver wrist | 0/169 | 13/169 | 0/169 | 88.8% / 79.0% |

The denominator includes frames after the hands left. Each wrist returned only the far hand.
Full-frame diagnostics logged 26 pose rejections across the two IDs on Blue and 30 on Silver.
Oblique views and pose/depth filtering matter at this grasp geometry. The earlier one-camera-at-a-time
[live check](../2026-11-18_glove_test/README.md#live-check-of-all-three-cameras-1612-1621) measured
0.170–0.227 s p95 glove age with more favorable views. Use a synchronized bag to measure fused
coverage, actual dropout gaps, and end-to-end age during handling.

The deployed [baseline overlay](images/baseline.jpg) selects none of either overhead probe, even
with abundant registered depth. Its normalized ROI ends at x ≈ 1075 and begins at y = 540, missing
much of the real work area. In the existing segmenter, a candidate ROI of
`[0.48, 0.25, 0.80, 0.65]` includes 87% and 68% of the steady and sagged probe pixels. With the
same ROI and S ≤ 60, V ≤ 90, those fractions rise above 99% on the two probes. Both settings
were tried on the recorded scene. Background false positives and coverage on independent clips still
need scoring. The wrist masks keep 91% and 95% of the steady probe pixels but only
18% and 1% of the sagged probes (Blue, Silver). They also select chairs, fixtures and floor features.
These figures describe the original diagnostic rectangles, which can include background, glare or
hardware. Whole-sheet precision, recall and depth completeness still require visible-cloth labels.
The [machine-readable results](results.json) include rectangles, ranges, baseline mask pixels,
depth samples and boundary weights.

The exploratory overlays show [wider HSV limits](images/sheet_masks_wide.jpg), then
[colour and geometry with invalid depth kept](images/sheet_masks_gated.jpg), and
[colour and geometry with a closed depth gate](images/sheet_masks_gated2.jpg).
The last uses overhead HSV S ≤ 60, V ≤ 90 with a world box
`[-0.50, -0.50, 0.30] .. [0.50, 0.45, 1.20]` m, and wrist S ≤ 70, V ≤ 150 with a
0.10–1.25 m depth band. It closes the depth gate with a 31 px kernel and keeps connected
components of at least 300 px. The overhead view improves visually. The wrist gates clip RGB-visible
cloth at the narrower depth field of view, and Silver's sagged frame still has speckled holes.
Blue's floor/image-bottom probe also contains near returns, so a wrist depth band alone is insufficient.
Trying a V ≤ 60 branch independent of saturation was proposed after examining Silver's speckle patch.
The following trial evaluates those settings through the implemented perception path.

## Labelled workspace trial

Visible-cloth polygons were drawn on frames 7 (steady), 27 (sagged), and 40 (released) from each
camera. The [annotations](workspace_labels.json) use 480 × 270 coordinates. The replay scales them
to raw RGB resolution and excludes a 12 px band near each outline before computing precision and
recall. The thresholds came from other frames in the same recording. These scores describe a
single scene with approximate labels.

| Camera | Candidate | Steady precision / recall | Sagged precision / recall |
| --- | --- | ---: | ---: |
| Overhead | Image ROI, HSV, world box | 96.6% / 92.2% | 94.7% / 90.1% |
| Blue wrist | HSV, V ≤ 60 branch, optical-depth band | 99.8% / 88.2% | 98.0% / 88.7% |
| Silver wrist | HSV, V ≤ 60 branch, optical-depth band | 99.6% / 88.8% | 96.8% / 80.5% |

On the sagged wrist frames, the dark branch raised recall from 79.9% to 88.7% on Blue and
75.6% to 80.5% on Silver. The registered depth field of view still clips visible cloth at both
wrists. The labelled wrist interiors had zero depth on the released frame, and the gated masks
had zero recall there. The thin overhead outline on frame 40 had no interior after applying
the annotation tolerance. Background and clip examples from a separate run are the next
validation data.

The [comparison image](images/workspace_comparison.jpg) and [results](workspace_results.json)
show the original baseline, updated colour and ROI, geometry, and the dark branch. The results
include retained depth and supported contours. The [built ROS node replay](diagnostics/workspace_ros_replay.txt)
used the steady and sagged frames from each camera with the staged parameter file. All six
produced observations. The warmed callbacks took 50–70 ms on the compute laptop. The first
overhead callback, including the ray cache, took 186 ms. Measure capture-to-publication age on
the camera hosts during the next trial.

The [candidate overlay](../../cell/perception_workspace.yaml) is for the stationary stand-in test.
For a perception-only test on the selected camera host, use the absolute path to its deployed
cell directory and pick the matching `camera` value:

```bash
cell=/absolute/path/to/the/deployed/cell
ros2 launch fabric_cell camera_host.launch.py camera:=overhead \
  cell_dir:="$cell" perception_overrides:="$cell/perception_workspace.yaml"
```

The overhead box uses the measured world-to-camera TF chain. Bring up that existing calibration
before testing the box. Robot and glove calibration still gate the measured-cell estimator run.

## Cartesian attempt and final state

MoveIt preflight accepted the inward targets (1.5 cm Blue, 1.3 cm Silver) at the recorded
orientation. Both helpers were restarted at 17:30 without changing the pose. The
[17:34 diagnostic](diagnostics/driver_input_1734.txt) at each driver's input then saw
approximately 100 Hz of nonzero commands at ~3 cm/s for six seconds,
with Fabric Follow selected and deadman held. Joint change over the window was zero on both arms.
Across all six attempt CSVs the largest observed tool movement was 0.001 mm at the stored precision.
This isolates the observed failure downstream of the guard output on this test path. A passing
MoveIt preflight establishes model feasibility. It does not establish that the Kinova Cartesian
controller accepts this pose. The firmware protection/singularity theory remains untested.

The operator deferred the inset. All pending move scripts were cancelled. The interrupted JSON
reports are empty objects; their CSVs and text logs retain the evidence. The read-only
[`kstatus.cpp`](../2026-11-18_kinova_status/kstatus.cpp) was compiled on Blue and has no device
output from this session. The measured 82.9 cm tool-to-tool gap cannot establish the actual
clip-to-clip slack until H15 measures the clip contact points.

The [17:51 read-only verification](diagnostics/service_check.txt) found both helper stacks, the NUC
camera stack, the tablet GUI and Silver's hosted Blue force reader active. All `fabric-cell` and
`fabric-bridge` services were inactive. Both helpers had no grasp script, reported the deadman
released and mode disabled, and sent only zero Cartesian commands over six seconds.

## Reproduce the audit

From `oarbots`, with OpenCV, NumPy, PyYAML and the Jazzy Python environment used by `glove_check.py`:

```bash
source /opt/ros/jazzy/setup.bash
session=experiments/hardware/sessions/2026-11-18_sheet_capture
capture=/home/taha/fabric_hw_data/2026-11-18_sheet_capture
PYTHONPATH="scripts/hw:src/fabric_perception:src/human_hand_tracking:${PYTHONPATH:-}" \
  python3 "$session/score_capture.py" "$capture"
PYTHONPATH="scripts/hw:src/fabric_perception:src/human_hand_tracking:${PYTHONPATH:-}" \
  python3 "$session/score_capture.py" "$capture" --workspace
(cd "$capture" && sha256sum --quiet -c "$OLDPWD/$session/raw_files.sha256")
```

The two raw archives passed their manifest checks, and rescoring the external sheet archive
reproduced `results.json`. The existing perception, hand tracking, guard and bridge suites passed
53 tests. The twin and estimator numerics suites passed 27 tests with the laptop GPU available.
Both configuration loaders accepted the measured stand-in density.
