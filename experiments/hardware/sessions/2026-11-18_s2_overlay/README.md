# S2 estimated cloth on the live overhead camera

The hardware run reused the simulation's `fabric_sim_cell` marker publisher and RViz Camera
display. The display projected the estimated mesh onto the real overhead RGB feed using the
surveyed overhead pose and the camera driver's intrinsics. The wrist Camera panel was disabled
for this run. Both robot arms stayed at the grasp pose with nominal simulation grip offsets.
The twin and command publishing were disabled. [Run assumptions](assumptions.json)

## Held-sheet result

[Steady and moving overlays](images/overhead_overlay.png) show the blue mesh following the
main shape of the sheet through the hold and slow corner motion. The moving view shows visible
mismatch in the upper and lower edges. RViz displays the most recent mesh over the camera
image. The images support visual inspection of the fit, with no measured edge error or
frame-by-frame synchronization claim.

| Input to estimator during the 52 s camera run | Cloth observations | Glove messages | Observation age p95 |
| --- | ---: | ---: | ---: |
| Blue wrist | 261 | 778 | 146 ms |
| Silver wrist | 241 | 734 | 170 ms |
| Overhead | 0 | 0 | — |

The overhead image was available to RViz. The overhead perception node rejected its image pairs
on its timestamp checks, so the estimated cloth shown here relied on the wrists. The overlaid
overhead view was therefore separate from the observations that produced the estimate.
[Per-camera observations and diagnostics](live/cameras/)

## Overhead timestamp recovery and preview check

With the sheet set down, overhead perception returned zero observations with the preview off
and with it on. Raw RGB was already a median 375 ms old when sampled on the NUC, and aligned
depth was 368 ms old. Both exceeded the 250 ms perception limit. The NUC's system clock was
within 2 ms of the compute laptop. Restarting just the NUC's overhead camera service restored
fresh input. The source of the accumulated camera timestamp age remains open.
**Update 21:40:** found. RViz showed the full-resolution overhead image on the laptop, which fills
the NUC's gigabit link; the driver backs up until restarted. Reproduced and fixed in `aee15a9` (preview
relay in the NUC session, RViz on the preview); see the lab log's evening check.
[Frame age](live/preview/timestamp_age.txt) · [Host clocks](live/preview/clock_offsets.txt)

The optional [NUC preview relay](preview_relay.py) (now `scripts/hw/preview_relay.py`, started by the NUC session) decimates RGB to 640 × 360 at up to 5 Hz. It
preserves the capture stamp and rescales the intrinsics for the same
[RViz Camera display](cell/fabric_cell_preview.rviz). Overhead perception still consumes full
resolution on the NUC.

| Overhead after camera restart | Window | Observations | Rate | Publication age p95 |
| --- | ---: | ---: | ---: | ---: |
| Preview off | 15 s | 74 | 4.93 Hz | 136 ms |
| Preview on | 25 s | 124 | 4.96 Hz | 141 ms |

[Restart comparison](live/preview/restart_test.log) ·
[RViz preview smoke check](images/rviz_preview_workspace.png)

The preview comparison used the workspace after the held trial. The next held run should use
the preview and check all three camera inputs before the steady cue. Continue tracking the
overhead frame age over a longer window before S3. Live alignment has been inspected, and the
100 ms S2 timing target and quantitative shape error are still open.

The later [operator repeat](repeat_20261118_195112/README.md) reproduced the overhead timestamp
rejection and documented two hand-corner jumps over 60 cm. The steady and moving images from
the original run illustrate the overlay display. The repeat establishes its live limitations.

## Repeat the overlay

On the Ethernet-connected compute laptop, run the same estimator, calibration publisher and
RViz display with a fresh recording directory. Keep the arms at the stationary grasp pose.

```bash
cd ~/ros2_colcon_ws/src/oarbots
source scripts/hw/laptop_env.sh

session="$PWD/experiments/hardware/sessions/2026-11-18_s2_overlay"
run=$(mktemp -d "$session/repeat_$(date +%Y%m%d_%H%M%S)_XXXXXX")
mkdir -p "$run/estimate" "$run/images" "$run/ros_log"
cp -a "$session/cell" "$run/cell"
cp "$session/assumptions.json" "$run/"
printf 'Repeat output directory %s\n' "$run"

export QT_QPA_PLATFORM=xcb
export WARP_CACHE_PATH=/tmp/oarbots-fabric-audit-gpu
export PYTHONPATH="$PWD/src/fabric_state_estimator:$PWD/src/human_hand_tracking:$PWD/src/fabric_sim_cell:${PYTHONPATH:-}"
export ROS_LOG_DIR="$run/ros_log"

ros2 launch "$PWD/src/fabric_cell/launch/compute.launch.py" \
  cell_dir:="$run/cell" \
  nominal_grips:=true twin:=false bank:=false commands:=false \
  rviz:=true record:="$run/estimate" 2>&1 | tee "$run/compute.log"
```

If only the camera drivers are running, start these existing perception runners in a second
laptop terminal. Set `run` to the output directory printed above. Use one perception process
per camera. This records for three minutes and collects the camera logs as each runner exits.

```bash
run='/paste/the/printed/repeat/path/here'
lssh=/media/taha/ExtermeLinux/claude-1000/lab/lssh
trial="${run##*/}"

run_camera() (
  set -o pipefail
  host=$1
  camera=$2
  output="$run/cameras/$camera"
  mkdir -p "$output"
  status=0
  "$lssh" -o StrictHostKeyChecking=yes "$host" "bash -s -- $camera $trial" <<'REMOTE' \
    2>&1 | tee "$output/live.log" || status=$?
source /opt/ros/jazzy/setup.bash
source ~/fabric_ws/install/setup.bash
source ~/fabric_ws/scripts/hw/lib.sh
fabric_export_ros_env "$HOME/fabric_ws/run/cyclonedds.xml"
cd ~/fabric_ws/run/stationary_camera_trial/overlay || exit 1
export PYTHONPATH="$PWD/src/fabric_perception:${PYTHONPATH:-}"
python3 measure_live.py --camera "$1" --cell experiments/hardware/cell \
  --out "../$2" --seconds 180
REMOTE
  "$lssh" -o StrictHostKeyChecking=yes "$host" \
    "cd ~/fabric_ws/run/stationary_camera_trial/$trial && tar -cf - observations.jsonl gloves.jsonl summary.json" \
    | tar -xf - -C "$output" || exit $?
  exit "$status"
)

run_camera nuc@192.168.1.90 overhead &
overhead_pid=$!
run_camera oarbot_blue@192.168.1.71 blue_wrist &
blue_pid=$!
run_camera oarbot_silver@192.168.1.110 silver_wrist &
silver_pid=$!
for pid in "$overhead_pid" "$blue_pid" "$silver_pid"; do
  wait "$pid" || printf 'Camera runner %s reported an error. Check its live.log.\n' "$pid" >&2
done
```

For the lighter feed, use the following command in another laptop terminal, then set the topic
in the RViz **Overhead camera + estimate** display to `/nuc/preview/rgb/image_raw`.

```bash
cd ~/ros2_colcon_ws/src/oarbots
LAB_SSH=/media/taha/ExtermeLinux/claude-1000/lab/lssh \
  scripts/hw/lab.sh session nuc exec python3 \
  fabric_ws/run/stationary_camera_trial/overlay_preview/preview_relay.py --seconds 240
```

Save the steady and moving RViz screenshots in `$run/images`, and note the cue times with
the trial. Stop the compute launch with **Ctrl+C** to flush its recordings. Check the three
camera summaries and keep the capture, logs and any settings changes together in the repeat
folder under the lab capture archive.

## Saved evidence

- [Steady RViz frame](images/rviz_steady.png), [moving RViz frame](images/rviz_moving.png),
  and additional movement frames in [images](images/).
- The [estimate NPZ](live/estimate.npz) contains 514 saved states. The hand states, estimator
  diagnostics, and [operator cues](live/phase_markers.txt) are in [live](live/).
  The overhead recording for this run consists of the RViz screenshots.
- [Camera and RViz configuration](cell/) and [launch log](live/compute.log).
- The [Blue](live/blue_postflight.log) and [Silver](live/silver_postflight.log) arm checks measured
  zero nonzero velocity messages and zero joint change over six seconds. Both arms reported
  disabled control modes with the deadman released.
- [SHA-256 manifest](recordings.sha256) for the session archive in the lab captures directory
  under `2026-11-18_s2_overlay`.
