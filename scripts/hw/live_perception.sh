#!/usr/bin/env bash
# Perception for one camera, on the camera's own computer, with the cell's settings plus the stand-in
# workspace profile (experiments/hardware/cell/perception_workspace.yaml), for a fixed time. Publishes the
# usual fabric/observations and fabric/gloves for the estimator and logs them (observations.jsonl,
# gloves.jsonl, summary.json). This is the runner the S2 runs used (measure_live.py); for the overhead it
# sets the surveyed world transform itself, so the workspace box needs no calibration publisher.
#   scripts/hw/lab.sh session nuc    exec bash fabric_ws/scripts/hw/live_perception.sh overhead     900
#   scripts/hw/lab.sh session blue   exec bash fabric_ws/scripts/hw/live_perception.sh blue_wrist   900
#   scripts/hw/lab.sh session silver exec bash fabric_ws/scripts/hw/live_perception.sh silver_wrist 900
# It stops by itself after the given seconds; to stop it earlier:
#   scripts/hw/lab.sh session <host> exec pkill -INT -f measure_live.py
set -eo pipefail
camera="${1:?usage: live_perception.sh <overhead|blue_wrist|silver_wrist> [seconds] [out_dir]}"
seconds="${2:-900}"
ws="$HOME/fabric_ws"
out="${3:-$ws/run/live_perception/${camera}_$(date +%Y%m%d_%H%M%S)}"
export PYTHONPATH="$ws/scripts/hw:${PYTHONPATH:-}"
echo "Perception for $camera for ${seconds} s; logs in $out"
exec python3 "$ws/experiments/hardware/sessions/2026-11-18_stationary_camera_trial/measure_live.py" \
    --camera "$camera" --cell "$ws/experiments/hardware/cell" --out "$out" --seconds "$seconds"
