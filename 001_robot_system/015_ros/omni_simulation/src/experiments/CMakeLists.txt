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
add_executable(exp1 src/exp1.cpp)
ament_target_dependencies(exp1 rclcpp geometry_msgs std_msgs)

add_executable(exp2 src/exp2.cpp)
ament_target_dependencies(exp2 rclcpp geometry_msgs std_msgs Eigen3)

add_executable(exp3 src/exp3.cpp)
ament_target_dependencies(exp3 rclcpp sensor_msgs std_msgs)

# nodes
install(TARGETS
  exp1
  exp2
  exp3

  DESTINATION lib/${PROJECT_NAME}/
)

# directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
