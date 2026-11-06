cmake_minimum_required(VERSION 3.8)
project(rokubi_force_torque_publisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(rokubi_force_torque_publisher
  src/rokubi_force_torque_publisher.cpp
  src/BotaForceTorqueSensorComm.cpp)
ament_target_dependencies(
  rokubi_force_torque_publisher
  rclcpp
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(rokubi_force_torque_publisher
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Compensation of a raw wrench topic, without the serial port (simulated cells, recordings).
add_executable(rokubi_force_torque_compensator src/rokubi_force_torque_compensator.cpp)
ament_target_dependencies(
  rokubi_force_torque_compensator
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(rokubi_force_torque_compensator
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(TARGETS
  rokubi_force_torque_publisher
  rokubi_force_torque_compensator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_serial test/test_serial.cpp src/BotaForceTorqueSensorComm.cpp)
  target_include_directories(test_serial PRIVATE include)
  ament_add_gtest(test_compensation test/test_compensation.cpp)
  target_include_directories(test_compensation PRIVATE include)
  ament_target_dependencies(test_compensation tf2)
endif()

ament_package()
