#include "rokubi_force_torque_publisher/rokubi_force_torque_publisher.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <system_error>

using SteadyClock = std::chrono::steady_clock;

tf2::Vector3 RokubiForceTorquePublisher::vector_parameter(const std::string& name) {
  auto v = declare_parameter<std::vector<double>>(name, {0.0, 0.0, 0.0});
  if (v.size() != 3 || !std::all_of(v.begin(), v.end(), [](double x){return std::isfinite(x);}))
    throw std::invalid_argument(name + " must contain three finite values");
  return {v[0], v[1], v[2]};
}

RokubiForceTorquePublisher::RokubiForceTorquePublisher() : Node("rokubi_force_torque_publisher") {
  prefix_ = declare_parameter<std::string>("tf_prefix", "");
  if (!prefix_.empty() && (prefix_.front() == '/' || prefix_.back() != '/'))
    throw std::invalid_argument("tf_prefix must be empty or have only a trailing slash");
  raw_frame_ = prefix_ + "j2n6s300_ft_raw";
  calibrated_frame_ = prefix_ + "j2n6s300_ft_measurement";
  // base_link assumes a level base. Set world after calibrated world TF is available on slopes.
  gravity_frame_ = declare_parameter<std::string>("gravity_frame", prefix_ + "base_link");
  mass_ = declare_parameter<double>("end_effector_mass", 0.0);
  max_age_ = declare_parameter<double>("max_sample_age_sec", 0.1);
  if (!std::isfinite(mass_) || mass_ < 0 || !std::isfinite(max_age_) || max_age_ <= 0)
    throw std::invalid_argument("Mass must be nonnegative and max_sample_age_sec positive");
  com_ = vector_parameter("center_of_mass_vector");
  force_bias_ = vector_parameter("force_bias_vector");
  torque_bias_ = vector_parameter("torque_bias_vector");
  blue_ = prefix_ == "oarbot_blue/";
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  raw_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
    declare_parameter<std::string>("raw_publish_topic", "force_torque_raw"), rclcpp::SensorDataQoS());
  calibrated_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
    declare_parameter<std::string>("calibrated_publish_topic", "force_torque_calibrated"), rclcpp::SensorDataQoS());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  const auto device = declare_parameter<std::string>("serial_port", "/dev/sensor_bota_rokubi_ft_0");
  serial_.value = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (serial_.value < 0) throw std::system_error(errno, std::generic_category(), "Opening " + device);
  termios tty{};
  if (tcgetattr(serial_.value, &tty) != 0) throw std::system_error(errno, std::generic_category(), "tcgetattr");
  cfmakeraw(&tty);
  tty.c_cflag = (tty.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  cfsetispeed(&tty, B460800);
  cfsetospeed(&tty, B460800);
  if (tcsetattr(serial_.value, TCSANOW, &tty) != 0) throw std::system_error(errno, std::generic_category(), "tcsetattr");
  serial_struct info{};
  if (ioctl(serial_.value, TIOCGSERIAL, &info) == 0) {
    info.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(serial_.value, TIOCSSERIAL, &info) != 0)
      RCLCPP_WARN(get_logger(), "Could not enable FTDI low latency; check measured sample latency");
  }
  sensor_ = std::make_unique<BotaForceTorqueSensorComm>(serial_.value);
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]{poll();});
  diagnostics_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]{diagnostics();});
}

void RokubiForceTorquePublisher::poll() {
  bool valid = false, drained = false;
  BotaForceTorqueSensorComm::Data latest;
  // Drain a bounded batch. Only the newest decoded sample gets a host receipt stamp.
  for (int i = 0; i < 256; ++i) {
    const auto result = sensor_->readFrame();
    if (result == BotaForceTorqueSensorComm::NO_FRAME) {drained = true; break;}
    if (result != BotaForceTorqueSensorComm::VALID_FRAME) continue;
    sensor_status_ = sensor_->frame.data.status;
    if (sensor_status_ != 0 || !std::all_of(sensor_->frame.data.forces.begin(), sensor_->frame.data.forces.end(),
        [](float value){return std::isfinite(value);})) {
      ++invalid_;
      valid = false;
      continue;
    }
    if (valid) ++coalesced_;
    latest = sensor_->frame.data;
    valid = true;
  }
  if (valid && drained) {
    last_receive_ = SteadyClock::now();
    ++received_;
    sensor_timestamp_ = latest.timestamp;
    geometry_msgs::msg::WrenchStamped msg;
    // Device ticks are diagnostic only until the device clock is calibrated to ROS time.
    msg.header.stamp = now();
    msg.header.frame_id = raw_frame_;
    msg.wrench.force.x = latest.forces[0]; msg.wrench.force.y = latest.forces[1]; msg.wrench.force.z = latest.forces[2];
    msg.wrench.torque.x = latest.forces[3]; msg.wrench.torque.y = latest.forces[4]; msg.wrench.torque.z = latest.forces[5];
    raw_pub_->publish(msg);
    if (pending_.size() >= 32) {pending_.pop_front(); ++tf_dropped_;}
    pending_.push_back({msg, last_receive_});
  }
  while (!pending_.empty()) {
    const auto& sample = pending_.front();
    if (std::chrono::duration<double>(SteadyClock::now() - sample.received).count() > max_age_) {
      pending_.pop_front(); ++tf_dropped_; continue;
    }
    try {
      auto transform = tf_buffer_->lookupTransform(calibrated_frame_, gravity_frame_, rclcpp::Time(sample.msg.header.stamp));
      tf2::Transform tf;
      tf2::fromMsg(transform.transform, tf);
      const auto gravity = tf.getBasis() * tf2::Vector3(0, 0, -9.80665);
      tf2::Vector3 f, t;
      tf2::fromMsg(sample.msg.wrench.force, f);
      tf2::fromMsg(sample.msg.wrench.torque, t);
      // Same sensor origin; rotate axes only. URDF documents these two frames.
      f = tf2::Vector3(blue_ ? f.x() : -f.x(), blue_ ? -f.y() : f.y(), -f.z());
      t = tf2::Vector3(blue_ ? t.x() : -t.x(), blue_ ? -t.y() : t.y(), -t.z());
      auto output = sample.msg;
      output.header.frame_id = calibrated_frame_;
      output.wrench.force = tf2::toMsg(f - mass_*gravity - force_bias_);
      output.wrench.torque = tf2::toMsg(t - mass_*com_.cross(gravity) - torque_bias_);
      calibrated_pub_->publish(output);
      pending_.pop_front();
    } catch (const tf2::TransformException&) {
      // Wait for joint state TF at the acquisition time, never substitute a latest transform.
      break;
    }
  }
}

void RokubiForceTorquePublisher::diagnostics() {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(get_namespace()) + "/rokubi";
  status.hardware_id = raw_frame_;
  const auto age = received_ ? std::chrono::duration<double>(SteadyClock::now() - last_receive_).count() : -1.0;
  status.level = (age < 0 || age > max_age_ || sensor_status_) ? 2 : (pending_.empty() ? 0 : 1);
  status.message = status.level == 2 ? "No fresh valid sensor data" : (status.level == 1 ? "Waiting for timestamped TF" : "Receiving");
  auto add = [&](const std::string& key, const auto& value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = std::to_string(value); status.values.push_back(kv);
  };
  add("receipt_age_sec", age); add("crc_errors", sensor_->get_crc_count()); add("invalid_frames", invalid_);
  add("sensor_status", sensor_status_); add("sensor_ticks", sensor_timestamp_); add("tf_dropped", tf_dropped_); add("coalesced_frames", coalesced_);
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = EXIT_SUCCESS;
  try {rclcpp::spin(std::make_shared<RokubiForceTorquePublisher>());}
  catch (const std::exception& e) {RCLCPP_ERROR(rclcpp::get_logger("rokubi"), "%s", e.what()); result = EXIT_FAILURE;}
  rclcpp::shutdown();
  return result;
}
