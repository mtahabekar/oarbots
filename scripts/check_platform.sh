#!/usr/bin/env bash
# Run after a build and after sourcing its install/setup.bash. Never launches hardware drivers.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID=197
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/oarbots-test-logs}"
cd "$repo_dir"
python3 -m pytest -q tests src/oarbot_runtime/test src/force_torque_calibration/test/test_calibration.py src/fabric_perception/test src/human_hand_tracking/test
