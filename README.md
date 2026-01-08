# state estimation for f1tenth racing stack(misys lab)


# Demo
![Demo GIF](assets/mapping_teras_x15.gif)
mapping demo(15x speed)

![Demo GIF](assets/ndt_ekf_localization_x3.gif)
localization demo(ndt scan matcher + ekf localizer, 3x speed)

![Demo GIF](assets/real_world_car.gif)
real world car drive demo(1.5x speed)


# Target HW
- NVIDIA AGX Orin
- Livox MID 360 (3D LiDAR)
- VESC mkIV


# Environment
- Ubuntu 22.04
- ROS2 humble


# Referenced open source
- https://github.com/rsasaki0109/lidarslam_ros2
- https://github.com/TUM-AVS/RoboRacer-3DLiDAR
- https://github.com/Livox-SDK/Livox-SDK2
- https://github.com/Livox-SDK/livox_ros_driver2
- https://autowarefoundation.github.io/autoware.universe_planning/pr-5583/localization/gyro_odometer/
- https://autowarefoundation.github.io/autoware.universe_planning/pr-5583/localization/ekf_localizer/
