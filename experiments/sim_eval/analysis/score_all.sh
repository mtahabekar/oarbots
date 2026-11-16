#!/usr/bin/env bash
# Plant-mechanics pull (CPU, slow) and scoring for every closed-loop run that lacks them, in parallel.
#   score_all.sh [PATTERN] [JOBS]
cd "$(dirname "$(realpath "$0")")"
source /opt/ros/jazzy/setup.bash; source /home/taha/ros2_colcon_ws/install/setup.bash
export PYTHONPATH=.:../../../src/fabric_twin:../../../src/fabric_sim_cell:$PYTHONPATH CUDA_VISIBLE_DEVICES=
DATA=${SIM_EVAL_DATA:-/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval}
ls -d $DATA/closed/${1:-cl_*} $DATA/full/full_* 2>/dev/null | while read -r run; do
  [[ -f "$run/manifest.json" && -f "$run/truth.npz" ]] || continue
  echo "$run"
done | xargs -P "${2:-5}" -I{} bash -c '[[ -f {}/true_tension.npz ]] || nice -n 19 ~/venvs/fabric_ros/bin/python true_tension.py {} > {}/true_tension.log 2>&1; nice -n 19 ~/venvs/fabric_ros/bin/python score_closed.py {} 2>&1 | tail -1'
