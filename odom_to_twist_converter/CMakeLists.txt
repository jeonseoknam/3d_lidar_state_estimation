cmake_minimum_required(VERSION 3.8)
project(odom_to_twist_converter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(odom_to_twist_converter src/odom_to_twist_converter.cpp)
ament_target_dependencies(odom_to_twist_converter rclcpp nav_msgs geometry_msgs)

install(TARGETS
  odom_to_twist_converter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}/
)
  
ament_package()

