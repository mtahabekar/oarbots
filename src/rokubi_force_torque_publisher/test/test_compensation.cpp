#include <gtest/gtest.h>

#include "rokubi_force_torque_publisher/compensation.hpp"

namespace {
void expect_near(const tf2::Vector3& a, const tf2::Vector3& b) {
  EXPECT_NEAR(a.x(), b.x(), 1e-12);
  EXPECT_NEAR(a.y(), b.y(), 1e-12);
  EXPECT_NEAR(a.z(), b.z(), 1e-12);
}
}  // namespace

TEST(Compensation, AxisFlipsMatchTheTwoMountings) {
  expect_near(rokubi::to_measurement({1, 2, 3}, true), {1, -2, -3});
  expect_near(rokubi::to_measurement({1, 2, 3}, false), {-1, 2, -3});
}

TEST(Compensation, StaticToolLoadCancels) {
  rokubi::Calibration c;
  c.mass = 1.5;
  c.com = {0.01, -0.02, -0.05};
  const tf2::Vector3 gravity(0.0, 0.0, -9.80665);
  // The tool's weight as the sensor measures it, expressed in raw axes for each robot.
  const tf2::Vector3 load = c.mass * gravity, moment = c.mass * c.com.cross(gravity);
  for (bool blue : {true, false}) {
    tf2::Vector3 raw_force = rokubi::to_measurement(load, blue), raw_torque = rokubi::to_measurement(moment, blue);
    tf2::Vector3 force, torque;
    rokubi::compensate(raw_force, raw_torque, blue, gravity, c, force, torque);
    expect_near(force, {0, 0, 0});
    expect_near(torque, {0, 0, 0});
  }
}

TEST(Compensation, ExternalLoadAndBiasRemain) {
  rokubi::Calibration c;
  c.mass = 1.0;
  c.force_bias = {0.1, 0.0, 0.0};
  const tf2::Vector3 gravity(0.0, 0.0, -9.80665), cloth(0.5, -0.2, -2.0);
  tf2::Vector3 force, torque;
  rokubi::compensate(rokubi::to_measurement(c.mass * gravity + cloth + c.force_bias, true), {0, 0, 0}, true, gravity, c,
                     force, torque);
  expect_near(force, cloth);
}
