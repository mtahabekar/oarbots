// Calibrated Rokubi wrench from a raw wrench topic: the serial driver's compensation without
// the serial port. Used with a simulated cell (or any recorded raw stream). Sample age is
// judged on the node's ROS clock, so it follows simulation time under use_sim_time.

#include "rokubi_force_torque_publisher/compensation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

using geometry_msgs::msg::WrenchStamped;

class RokubiForceTorqueCompensator : public rclcpp::Node {
public:
  RokubiForceTorqueCompensator() : Node("rokubi_force_torque_compensator") {
    prefix_ = declare_parameter<std::string>("tf_prefix", "");
    if (!prefix_.empty() && (prefix_.front() == '/' || prefix_.back() != '/'))
      throw std::invalid_argument("tf_prefix must be empty or have only a trailing slash");
    raw_frame_ = prefix_ + "j2n6s300_ft_raw";
    calibrated_frame_ = prefix_ + "j2n6s300_ft_measurement";
    gravity_frame_ = declare_parameter<std::string>("gravity_frame", prefix_ + "base_link");
    calibration_.mass = declare_parameter<double>("end_effector_mass", 0.0);
    max_age_ = declare_parameter<double>("max_sample_age_sec", 0.25);
    if (!std::isfinite(calibration_.mass) || calibration_.mass < 0 || !std::isfinite(max_age_) || max_age_ <= 0)
      throw std::invalid_argument("Mass must be nonnegative and max_sample_age_sec positive");
    calibration_.com = vector_parameter("center_of_mass_vector");
    calibration_.force_bias = vector_parameter("force_bias_vector");
    calibration_.torque_bias = vector_parameter("torque_bias_vector");
    blue_ = prefix_ == "oarbot_blue/";
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<WrenchStamped>(
      declare_parameter<std::string>("calibrated_publish_topic", "force_torque_calibrated"), rclcpp::SensorDataQoS());
    subscription_ = create_subscription<WrenchStamped>(
      declare_parameter<std::string>("raw_topic", "force_torque_raw"), rclcpp::SensorDataQoS(),
      [this](WrenchStamped::SharedPtr msg) {
        if (msg->header.frame_id != raw_frame_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping raw wrench in %s, expected %s",
                               msg->header.frame_id.c_str(), raw_frame_.c_str());
          return;
        }
        if (pending_.size() >= 64) {pending_.pop_front(); ++dropped_;}
        pending_.push_back(*msg);
        process();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {process();});
  }

private:
  std::string prefix_, raw_frame_, calibrated_frame_, gravity_frame_;
  rokubi::Calibration calibration_;
  double max_age_;
  bool blue_;
  uint64_t dropped_ = 0;
  std::deque<WrenchStamped> pending_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<WrenchStamped>::SharedPtr publisher_;
  rclcpp::Subscription<WrenchStamped>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::Vector3 vector_parameter(const std::string& name) {
    auto v = declare_parameter<std::vector<double>>(name, {0.0, 0.0, 0.0});
    if (v.size() != 3 || !std::all_of(v.begin(), v.end(), [](double x) {return std::isfinite(x);}))
      throw std::invalid_argument(name + " must contain three finite values");
    return {v[0], v[1], v[2]};
  }

  void process() {
    while (!pending_.empty()) {
      const auto& sample = pending_.front();
      const rclcpp::Time stamp(sample.header.stamp, get_clock()->get_clock_type());
      if ((now() - stamp).seconds() > max_age_) {pending_.pop_front(); ++dropped_; continue;}
      try {
        // Gravity at the sample time, never the latest transform.
        auto transform = tf_buffer_->lookupTransform(calibrated_frame_, gravity_frame_, stamp);
        tf2::Transform tf;
        tf2::fromMsg(transform.transform, tf);
        const auto gravity = tf.getBasis() * tf2::Vector3(0, 0, -9.80665);
        tf2::Vector3 raw_force, raw_torque, force, torque;
        tf2::fromMsg(sample.wrench.force, raw_force);
        tf2::fromMsg(sample.wrench.torque, raw_torque);
        rokubi::compensate(raw_force, raw_torque, blue_, gravity, calibration_, force, torque);
        WrenchStamped output = sample;
        output.header.frame_id = calibrated_frame_;
        output.wrench.force = tf2::toMsg(force);
        output.wrench.torque = tf2::toMsg(torque);
        publisher_->publish(output);
        pending_.pop_front();
      } catch (const tf2::TransformException&) {
        break;  // wait for joint-state TF at the sample time
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RokubiForceTorqueCompensator>());
  rclcpp::shutdown();
  return 0;
}
