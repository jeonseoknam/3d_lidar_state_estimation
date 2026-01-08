from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carstate_3d',
            executable='carstate_3d_node',
            name='carstate_3d',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'ekf_odom_topic': '/ekf_odom',
                'ekf_pose_topic': '/ekf_pose',
                'carstate_odom_topic': '/car_state/odom',
                'carstate_pose_topic': '/car_state/pose',
                'frenet_odom_topic': '/car_state/frenet/odom',
                'frenet_pose_topic': '/car_state/frenet/pose',
            }]
        )
    ])

