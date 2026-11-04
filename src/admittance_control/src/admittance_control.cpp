#include "admittance_control/admittance_control.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <cmath>

namespace {
using Clock = std::chrono::steady_clock;
double age(Clock::time_point time) {return std::chrono::duration<double>(Clock::now()-time).count();}
tf2::Vector3 step(tf2::Vector3 velocity, tf2::Vector3 force, double cutoff, double mass, double damping, double dt, double limit) {
  const auto norm = force.length();
  force = norm > cutoff ? force * ((norm-cutoff)/norm) : tf2::Vector3(0,0,0);
  const double decay = std::exp(-damping*dt/mass);
  velocity = decay*velocity + ((1-decay)/damping)*force;
  if (velocity.length() > limit) velocity *= limit/velocity.length();
  return velocity;
}
}

AdmittanceControl::AdmittanceControl() : Node("admittance_control") {
  prefix_ = declare_parameter<std::string>("tf_prefix", "");
  if (!prefix_.empty() && (prefix_.front() == '/' || prefix_.back() != '/'))
    throw std::invalid_argument("tf_prefix must be empty or have only a trailing slash");
  // Enabling is a renewable lease, never a persistent startup setting.
  if (declare_parameter<bool>("enable_initially", false))
    throw std::invalid_argument("Use the admittance_enable heartbeat to enable motion");
  translation_ = declare_parameter<bool>("enable_translation_initially", true);
  rotation_ = declare_parameter<bool>("enable_rotation_initially", false);
  angular_validated_ = declare_parameter<bool>("angular_mapping_validated", false);
  auto positive = [this](const std::string& name, double value) {
    auto x = declare_parameter<double>(name, value);
    if (!std::isfinite(x) || x <= 0) throw std::invalid_argument(name + " must be finite and positive");
    return x;
  };
  timeout_ = positive("enable_timeout_sec", 0.25); max_age_ = positive("wrench_timeout_sec", 0.1);
  mass_ = positive("virtual_mass_kg", 1.2); inertia_ = positive("virtual_inertia_kg_m2", 0.1);
  damping_ = positive("linear_damping_ns_m", 40.0); angular_damping_ = positive("angular_damping_nms_rad", 2.0);
  force_cutoff_ = positive("force_deadband_n", 5.0); torque_cutoff_ = positive("torque_deadband_nm", 0.35);
  linear_limit_ = positive("max_linear_mps", 0.1); angular_limit_ = positive("max_angular_rps", 0.3);
  buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  publisher_ = create_publisher<kinova_msgs::msg::PoseVelocity>("commands/admittance/arm", rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(max_age_)));
  enable_sub_ = create_subscription<std_msgs::msg::Bool>("admittance_enable", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg){
    last_enable_ = Clock::now(); enabled_ = msg->data; if (!enabled_) stop();
  });
  translation_sub_ = create_subscription<std_msgs::msg::Bool>("admittance_translation_enable", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg){translation_ = msg->data; if (!translation_) linear_.setZero();});
  rotation_sub_ = create_subscription<std_msgs::msg::Bool>("admittance_rotation_enable", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg){rotation_ = msg->data; if (!rotation_) angular_.setZero();});
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>("force_torque_calibrated", rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg){wrench(*msg);});
  watchdog_ = create_wall_timer(std::chrono::milliseconds(20), [this]{
    if (enabled_ && (age(last_enable_) > timeout_ || age(last_wrench_) > max_age_)) {enabled_ = false; stop();}
  });
}

void AdmittanceControl::stop() {
  linear_.setZero(); angular_.setZero();
  last_stamp_ = rclcpp::Time(0,0,RCL_ROS_TIME);
  publisher_->publish(kinova_msgs::msg::PoseVelocity());
}

void AdmittanceControl::wrench(const geometry_msgs::msg::WrenchStamped& msg) {
  if (!enabled_ || age(last_enable_) > timeout_) {stop(); return;}
  const rclcpp::Time stamp(msg.header.stamp);
  const double latency = (now()-stamp).seconds();
  const double values[] = {msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z};
  if (msg.header.frame_id.empty() || stamp.nanoseconds() == 0 || latency < -0.01 || latency > max_age_ ||
      !std::all_of(std::begin(values),std::end(values),[](double x){return std::isfinite(x);})) {stop(); return;}
  const double dt = last_stamp_.nanoseconds() == 0 ? 0.0 : (stamp-last_stamp_).seconds();
  if (dt < 0 || dt > max_age_) {stop(); return;}
  if (last_stamp_.nanoseconds() && dt == 0) return;
  try {
    // Linear velocity follows the arm API base axes, not the mobile base axes.
    const auto transform = buffer_->lookupTransform(prefix_ + "j2n6s300_link_base", msg.header.frame_id, stamp);
    tf2::Quaternion q; tf2::fromMsg(transform.transform.rotation, q);
    tf2::Vector3 force, torque;
    tf2::fromMsg(msg.wrench.force, force); tf2::fromMsg(msg.wrench.torque, torque);
    force = tf2::quatRotate(q, force);
    // Re-express moments in the historical connector axes, retaining the sensor origin.
    const auto torque_tf = buffer_->lookupTransform(prefix_ + "j2n6s300_ft_robot_side_connector", msg.header.frame_id, stamp);
    tf2::fromMsg(torque_tf.transform.rotation, q); torque = tf2::quatRotate(q, torque);
    linear_ = translation_ ? step(linear_, force, force_cutoff_, mass_, damping_, dt, linear_limit_) : tf2::Vector3(0,0,0);
    angular_ = rotation_ && angular_validated_ ? step(angular_, torque, torque_cutoff_, inertia_, angular_damping_, dt, angular_limit_) : tf2::Vector3(0,0,0);
    kinova_msgs::msg::PoseVelocity output;
    output.twist_linear_x = linear_.x(); output.twist_linear_y = linear_.y(); output.twist_linear_z = linear_.z();
    // Legacy API mapping requires physical verification; disabled by default.
    if (prefix_ == "oarbot_silver/") {
      output.twist_angular_x = angular_.y(); output.twist_angular_y = angular_.x(); output.twist_angular_z = -angular_.z();
    } else if (prefix_ == "oarbot_blue/") {
      output.twist_angular_x = -angular_.x(); output.twist_angular_y = -angular_.y(); output.twist_angular_z = angular_.z();
    }
    last_stamp_ = stamp; last_wrench_ = Clock::now();
    publisher_->publish(output);
  } catch (const tf2::TransformException& e) {
    stop();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Stopping: wrench TF unavailable: %s", e.what());
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = EXIT_SUCCESS;
  try {rclcpp::spin(std::make_shared<AdmittanceControl>());}
  catch (const std::exception& e) {RCLCPP_ERROR(rclcpp::get_logger("admittance_control"), "%s", e.what()); result = EXIT_FAILURE;}
  rclcpp::shutdown(); return result;
}
