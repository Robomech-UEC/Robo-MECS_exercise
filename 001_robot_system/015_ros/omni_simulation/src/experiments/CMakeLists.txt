cmake_minimum_required(VERSION 3.8)
project(experiments)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(
  PUBLIC $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)

# executables
add_executable(cmd_vel_check src/cmd_vel_check.cpp)
ament_target_dependencies(cmd_vel_check rclcpp geometry_msgs std_msgs)

add_executable(vel2wheels src/vel2wheels.cpp)
ament_target_dependencies(vel2wheels rclcpp geometry_msgs std_msgs Eigen3)

add_executable(quat2yaw src/quat2yaw.cpp)
ament_target_dependencies(quat2yaw rclcpp sensor_msgs std_msgs)

add_executable(pd_ctl src/pd_ctl.cpp)
ament_target_dependencies(pd_ctl rclcpp geometry_msgs std_msgs Eigen3)

# nodes
install(TARGETS
  cmd_vel_check
  vel2wheels
  quat2yaw
  pd_ctl

  DESTINATION lib/${PROJECT_NAME}/
)

# directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
