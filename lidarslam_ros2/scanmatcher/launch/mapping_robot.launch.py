import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mapping_param_dir = launch.substitutions.LaunchConfiguration(
        'mapping_param_dir',
        default=os.path.join(
            get_package_share_directory('scanmatcher'),
            'param',
            'mapping_robot.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[mapping_param_dir],
        remappings=[('/input_cloud','/livox/lidar'),('/imu','/livox/imu')],
        #remappings=[('/imu','/gpsimu_driver/imu_data')],# for imu debug
        output='screen'
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mapping_param_dir',
            default_value=mapping_param_dir,
            description='Full path to mapping parameter file to load'),
        mapping,
            ])