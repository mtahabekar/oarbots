#ifndef ADMITTANCE_CONTROL_HPP
#define ADMITTANCE_CONTROL_HPP
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "kinova_msgs/msg/pose_velocity.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include <chrono>

class AdmittanceControl : public rclcpp::Node {
public:
  AdmittanceControl();
private:
  std::string prefix_;
  bool enabled_ = false, translation_ = true, rotation_ = false, angular_validated_ = false;
  double timeout_, max_age_, mass_, inertia_, damping_, angular_damping_, force_cutoff_, torque_cutoff_, linear_limit_, angular_limit_;
  tf2::Vector3 linear_{0,0,0}, angular_{0,0,0};
  std::chrono::steady_clock::time_point last_enable_{}, last_wrench_{};
  rclcpp::Time last_stamp_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_, translation_sub_, rotation_sub_;
  rclcpp::Publisher<kinova_msgs::msg::PoseVelocity>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  void stop();
  void wrench(const geometry_msgs::msg::WrenchStamped& msg);
};
#endif
