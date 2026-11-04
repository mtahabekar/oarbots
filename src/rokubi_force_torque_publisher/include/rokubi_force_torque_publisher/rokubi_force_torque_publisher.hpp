#ifndef ROKUBI_FORCE_TORQUE_PUBLISHER
#define ROKUBI_FORCE_TORQUE_PUBLISHER

#include "rokubi_force_torque_publisher/BotaForceTorqueSensorComm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <chrono>
#include <deque>
#include <string>
#include <unistd.h>

class RokubiForceTorquePublisher : public rclcpp::Node {
public:
  RokubiForceTorquePublisher();
private:
  struct SerialDescriptor {int value = -1; ~SerialDescriptor() {if (value >= 0) ::close(value);}} serial_;
  std::string prefix_, raw_frame_, calibrated_frame_, gravity_frame_;
  double mass_, max_age_;
  tf2::Vector3 com_, force_bias_, torque_bias_;
  bool blue_;
  uint16_t sensor_status_ = 0;
  uint32_t sensor_timestamp_ = 0;
  uint64_t received_ = 0, invalid_ = 0, tf_dropped_ = 0, coalesced_ = 0;
  std::chrono::steady_clock::time_point last_receive_{};
  struct Pending {geometry_msgs::msg::WrenchStamped msg; std::chrono::steady_clock::time_point received;};
  std::deque<Pending> pending_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr raw_pub_, calibrated_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_, diagnostics_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<BotaForceTorqueSensorComm> sensor_;
  tf2::Vector3 vector_parameter(const std::string& name);
  void poll();
  void diagnostics();
};
#endif
