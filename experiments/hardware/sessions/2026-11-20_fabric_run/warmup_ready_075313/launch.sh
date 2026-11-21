#!/usr/bin/env bash
set -eo pipefail
cd /home/taha/ros2_colcon_ws/src/oarbots
source scripts/hw/laptop_env.sh
export QT_QPA_PLATFORM=xcb WARP_CACHE_PATH=/tmp/oarbots-fabric-audit-gpu DISPLAY="${DISPLAY:-:0}"
export PYTHONPATH="/home/taha/ros2_colcon_ws/src/oarbots/src/fabric_state_estimator:/home/taha/ros2_colcon_ws/src/oarbots/src/human_hand_tracking:/home/taha/ros2_colcon_ws/src/oarbots/src/fabric_sim_cell:${PYTHONPATH:-}"
ros2 launch /home/taha/ros2_colcon_ws/src/oarbots/src/fabric_cell/launch/compute.launch.py cell_dir:=/home/taha/ros2_colcon_ws/src/oarbots/experiments/hardware/sessions/2026-11-20_fabric_run/warmup_ready_075313/cell nominal_grips:=true twin:=true bank:=true commands:=false rviz:=true record:=/home/taha/ros2_colcon_ws/src/oarbots/experiments/hardware/sessions/2026-11-20_fabric_run/warmup_ready_075313 2>&1 | tee /home/taha/ros2_colcon_ws/src/oarbots/experiments/hardware/sessions/2026-11-20_fabric_run/warmup_ready_075313/compute.log
