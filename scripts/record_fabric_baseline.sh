#!/usr/bin/env bash
# Run on the recording computer after preflight, clocks and calibration are verified.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
recording_dir="${1:?Usage: record_fabric_baseline.sh /path/to/new/bag_directory}"
topics=(/tf /tf_static /aruco/markers /tablet_gui/deadman /tablet_gui/e_stop /fabric/hand_states /fabric/hand_tracking_status)
for camera in nuc oarbot_blue/azure_kinect oarbot_silver/azure_kinect; do
  for suffix in rgb/image_raw rgb/camera_info depth/image_raw depth/camera_info depth_to_rgb/image_raw depth_to_rgb/camera_info ir/image_raw ir/camera_info imu body_tracking_data fabric/observations fabric/gloves fabric/diagnostics fabric/robot_mask fabric/labels fabric/depth_valid; do
    topics+=("/$camera/$suffix")
  done
done
for robot in oarbot_blue oarbot_silver; do
  for suffix in joint_states force_torque_raw force_torque_calibrated diagnostics human/left_hand/pose human/right_hand/pose control_mode base_enable; do
    topics+=("/$robot/$suffix")
  done
done
exec ros2 bag record --storage mcap --output "$recording_dir" --qos-profile-overrides-path "$repo_dir/config/recording_qos.yaml" "${topics[@]}"
