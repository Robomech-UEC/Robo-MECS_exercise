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

add_executable(exp1 src/exp1.cpp)
ament_target_dependencies(exp1 rclcpp geometry_msgs std_msgs sensor_msgs)

# nodes
install(TARGETS
  exp1

  DESTINATION lib/${PROJECT_NAME}/
)

# directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
