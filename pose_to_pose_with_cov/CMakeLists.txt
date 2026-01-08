cmake_minimum_required(VERSION 3.8)
project(pose_to_pose_with_cov)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pose_to_pose_with_cov src/pose_to_pose_with_cov.cpp)
ament_target_dependencies(pose_to_pose_with_cov rclcpp geometry_msgs)
install(TARGETS pose_to_pose_with_cov DESTINATION lib/${PROJECT_NAME})


ament_package()
