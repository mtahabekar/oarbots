cmake_minimum_required(VERSION 3.28)
project(tf_relay)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf_relay
  src/tf_relay.cpp
)

ament_target_dependencies(
  tf_relay
  rclcpp
  tf2_msgs
  geometry_msgs
)

target_include_directories(tf_relay
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  tf_relay
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
