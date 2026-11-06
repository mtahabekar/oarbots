#ifndef ROKUBI_FORCE_TORQUE_COMPENSATION
#define ROKUBI_FORCE_TORQUE_COMPENSATION

// Static gravity and bias compensation of one Rokubi sample, shared by the serial driver and
// the standalone compensator so hardware and simulation use one implementation.

#include "tf2/LinearMath/Vector3.hpp"

namespace rokubi {

struct Calibration {
  double mass = 0.0;             // everything beyond the sensor, kg
  tf2::Vector3 com{0, 0, 0};     // its centre of mass in the measurement frame, m
  tf2::Vector3 force_bias{0, 0, 0};
  tf2::Vector3 torque_bias{0, 0, 0};
};

// Raw device axes to the documented measurement frame. Same sensor origin; the two robots'
// sensors are mounted differently (URDF j2n6s300_ft_raw vs j2n6s300_ft_measurement).
inline tf2::Vector3 to_measurement(const tf2::Vector3& v, bool blue) {
  return tf2::Vector3(blue ? v.x() : -v.x(), blue ? -v.y() : v.y(), -v.z());
}

// `gravity` is the gravity vector expressed in the measurement frame at the sample time.
inline void compensate(const tf2::Vector3& raw_force, const tf2::Vector3& raw_torque, bool blue,
                       const tf2::Vector3& gravity, const Calibration& c,
                       tf2::Vector3& force, tf2::Vector3& torque) {
  force = to_measurement(raw_force, blue) - c.mass * gravity - c.force_bias;
  torque = to_measurement(raw_torque, blue) - c.mass * c.com.cross(gravity) - c.torque_bias;
}

}  // namespace rokubi

#endif
