#!/usr/bin/env bash
# Perception for one camera, on the camera's own computer, with the cell's settings plus the stand-in
# workspace profile (experiments/hardware/cell/perception_workspace.yaml), for a fixed time. Publishes the
# usual fabric/observations and fabric/gloves for the estimator and logs them (observations.jsonl,
# gloves.jsonl, summary.json). The runner loads its surveyed world anchor into its own TF buffer.
# fabric_run.sh stages a copy of the runner and current cell profile on each host before starting it.
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
bundle="${4:-}"
if [ -n "$bundle" ]; then
    runner="$bundle/measure_live.py"
    cell="$bundle"
    export PYTHONPATH="$bundle:${PYTHONPATH:-}"
else
    runner="$ws/experiments/hardware/sessions/2026-11-18_stationary_camera_trial/measure_live.py"
    cell="$ws/experiments/hardware/cell"
    export PYTHONPATH="$ws/scripts/hw:${PYTHONPATH:-}"
fi
echo "Perception for $camera for ${seconds} s; logs in $out"
exec python3 "$runner" --camera "$camera" --cell "$cell" --out "$out" --seconds "$seconds"
