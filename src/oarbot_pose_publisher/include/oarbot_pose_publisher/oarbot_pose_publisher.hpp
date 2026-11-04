#ifndef OARBOT_POSE_PUBLISHER
#define OARBOT_POSE_PUBLISHER
#include "rclcpp/rclcpp.hpp"
#include "aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include <unordered_map>

class OarbotPosePublisher : public rclcpp::Node {
public:
  OarbotPosePublisher();
private:
  std::unordered_map<int64_t, std::string> tags_;
  double max_age_;
  rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr markers_sub_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  void markers(const aruco_interfaces::msg::ArucoMarkers& msg);
};
#endif
