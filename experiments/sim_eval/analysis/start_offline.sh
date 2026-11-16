#!/usr/bin/env bash
# Start CUDA MPS (small kernels from parallel processes then share the GPU) and run one offline study.
#   start_offline.sh s2|s3 [WORKERS]
export CUDA_MPS_PIPE_DIRECTORY=/tmp/claude-1000/mps_pipe CUDA_MPS_LOG_DIRECTORY=/tmp/claude-1000/mps_log
mkdir -p $CUDA_MPS_PIPE_DIRECTORY $CUDA_MPS_LOG_DIRECTORY
nvidia-cuda-mps-control -d 2>/dev/null || true   # already running is fine
cd "$(dirname "$(realpath "$0")")"
source /opt/ros/jazzy/setup.bash; source /home/taha/ros2_colcon_ws/install/setup.bash
export PYTHONPATH=.:../../../src/fabric_twin:../../../src/fabric_sim_cell:$PYTHONPATH
~/venvs/fabric_ros/bin/python run_offline.py "$1" --workers "${2:-8}"
STATUS=$?
echo quit | nvidia-cuda-mps-control
exit $STATUS
