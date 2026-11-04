#include "oarbot_pose_publisher/oarbot_pose_publisher.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <cmath>
#include <set>

OarbotPosePublisher::OarbotPosePublisher() : Node("oarbot_pose_publisher") {
  const auto topic = declare_parameter<std::string>("aruco_markers_topic", "/aruco/markers");
  // Kept for launch compatibility. An accelerometer cannot calibrate camera translation/yaw.
  declare_parameter<std::string>("kinect_imu_topic", "");
  const auto ids = declare_parameter<std::vector<int64_t>>("aruco_tag_ids", std::vector<int64_t>{});
  const auto names = declare_parameter<std::vector<std::string>>("oarbot_namespaces", std::vector<std::string>{});
  if (ids.empty() || ids.size() != names.size()) throw std::invalid_argument("Tag IDs and robot namespaces must have equal, nonzero size");
  std::set<std::string> seen_names;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (names[i].empty() || names[i].front() == '/' || names[i].back() != '/' || ids[i] < 0 ||
        !tags_.emplace(ids[i], names[i]).second || !seen_names.insert(names[i]).second)
      throw std::invalid_argument("Unique tag IDs/namespaces required; namespaces must have only a trailing slash");
  }
  max_age_ = declare_parameter<double>("max_observation_age_sec", 0.5);
  if (!std::isfinite(max_age_) || max_age_ <= 0) throw std::invalid_argument("max_observation_age_sec must be positive");
  buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  const auto publish_camera = declare_parameter<bool>("publish_camera_tf", false);
  const auto xyz = declare_parameter<std::vector<double>>("camera_translation", {0., 0., 0.});
  const auto rpy = declare_parameter<std::vector<double>>("camera_rpy", {0., 0., 0.});
  const auto world = declare_parameter<std::string>("world_frame", "world");
  const auto camera = declare_parameter<std::string>("camera_base_frame", "nuc/camera_base");
  if (xyz.size() != 3 || rpy.size() != 3 ||
      !std::all_of(xyz.begin(), xyz.end(), [](double x){return std::isfinite(x);}) ||
      !std::all_of(rpy.begin(), rpy.end(), [](double x){return std::isfinite(x);}) ||
      world.empty() || camera.empty() || world == camera || world.front() == '/' || camera.front() == '/')
    throw std::invalid_argument("Invalid camera extrinsics or frame names");
  if (publish_camera) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now(); transform.header.frame_id = world; transform.child_frame_id = camera;
    transform.transform.translation.x = xyz[0]; transform.transform.translation.y = xyz[1]; transform.transform.translation.z = xyz[2];
    tf2::Quaternion q; q.setRPY(rpy[0], rpy[1], rpy[2]);
    transform.transform.rotation = tf2::toMsg(q);
    static_broadcaster_->sendTransform(transform);
  } else {
    RCLCPP_WARN(get_logger(), "Supply calibrated world-to-camera TF externally, or set publish_camera_tf with measured extrinsics");
  }
  markers_sub_ = create_subscription<aruco_interfaces::msg::ArucoMarkers>(topic, rclcpp::SensorDataQoS(),
    [this](aruco_interfaces::msg::ArucoMarkers::ConstSharedPtr msg){markers(*msg);});
}

void OarbotPosePublisher::markers(const aruco_interfaces::msg::ArucoMarkers& msg) {
  const rclcpp::Time stamp(msg.header.stamp);
  const double age = (now() - stamp).seconds();
  if (msg.marker_ids.size() != msg.poses.size() || msg.header.frame_id.empty() ||
      msg.header.frame_id.front() == '/' || stamp.nanoseconds() == 0 || age < -0.05 || age > max_age_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Discarding malformed or stale ArUco observation");
    return;
  }
  std::set<int64_t> seen;
  for (size_t i = 0; i < msg.marker_ids.size(); ++i) {
    auto entry = tags_.find(msg.marker_ids[i]);
    if (entry == tags_.end() || !seen.insert(msg.marker_ids[i]).second) continue;
    const auto& p = msg.poses[i];
    const double values[] = {p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w};
    if (!std::all_of(std::begin(values), std::end(values), [](double x){return std::isfinite(x);})) continue;
    tf2::Quaternion q; tf2::fromMsg(p.orientation, q);
    if (q.length2() < 1e-12) continue;
    q.normalize();
    try {
      // Tag-to-base is rigid; still request the observation timestamp to maintain TF semantics.
      auto rigid = buffer_->lookupTransform(entry->second + "robot_arm_riser_aruco_mount_attach_link", entry->second + "base_link", stamp);
      tf2::Transform tag_to_base; tf2::fromMsg(rigid.transform, tag_to_base);
      tf2::Transform camera_to_tag(q, tf2::Vector3(p.position.x,p.position.y,p.position.z));
      geometry_msgs::msg::TransformStamped output;
      output.header = msg.header;
      output.child_frame_id = entry->second + "base_link";
      output.transform = tf2::toMsg(camera_to_tag * tag_to_base);
      broadcaster_->sendTransform(output);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ArUco TF unavailable: %s", e.what());
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = EXIT_SUCCESS;
  try {rclcpp::spin(std::make_shared<OarbotPosePublisher>());}
  catch (const std::exception& e) {RCLCPP_ERROR(rclcpp::get_logger("oarbot_pose_publisher"), "%s", e.what()); result = EXIT_FAILURE;}
  rclcpp::shutdown();
  return result;
}
