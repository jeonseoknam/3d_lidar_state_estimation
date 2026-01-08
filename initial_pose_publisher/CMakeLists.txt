cmake_minimum_required(VERSION 3.8)
project(initial_pose_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(lidar2initialpose src/lidar2initialpose.cpp)
ament_target_dependencies(lidar2initialpose rclcpp geometry_msgs std_srvs)

install(TARGETS lidar2initialpose
  DESTINATION lib/${PROJECT_NAME})

ament_package()
