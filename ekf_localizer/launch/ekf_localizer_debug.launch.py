import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Share directory for the ekf_localizer package
    ekf_localizer_share_dir = get_package_share_directory('ekf_localizer')

    # === Launch Arguments ===
    # XML의 <arg> 태그들을 Python 코드로 변환
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(ekf_localizer_share_dir, 'config', 'ekf_localizer.param.yaml'),
        description='Path to the EKF parameters file'
    )
    ekf_mode_arg = DeclareLaunchArgument(
        'ekf_mode',
        default_value='lidar',
        description='EKF operating mode'
    )
    ekf_node_name_arg = DeclareLaunchArgument(
        'ekf_node_name',
        default_value='ekf_localizer',
        description='Name of the EKF node'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # === Nodes ===
    # XML의 <node> 태그들을 Python 코드로 변환
    
    # Livox Pose 변환 노드
    pose_converter_node = Node(
        package='pose_to_pose_with_cov',
        executable='pose_to_pose_with_cov',
        name='pose_converter',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # EKF Localizer 노드 (GDB 연결)
    ekf_localizer_node = Node(
        package='ekf_localizer',
        executable='ekf_localizer',
        name=LaunchConfiguration('ekf_node_name'),
        output='screen',
        # GDB 연결
        prefix=['gdb -ex run --args'],
        parameters=[
            LaunchConfiguration('param_file'),
            {'ekf_mode': LaunchConfiguration('ekf_mode')},
            {'pose_frame_id': 'map'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('in_pose_with_covariance', '/ndt_pose_with_covariance'),
            ('in_twist_with_covariance', '/gyro_twist_with_covariance'),
            ('initialpose', 'initialpose'),
            ('trigger_node_srv', 'trigger_node')
        ]
    )

    # Initial Pose Publisher 노드
    initial_pose_publisher_node = Node(
        package='initial_pose_publisher',
        executable='lidar2initialpose',
        name='lidar2initialpose',
        parameters=[{'use_sim_time': False}]
    )

    # 모든 요소를 LaunchDescription에 담아 반환
    return LaunchDescription([
        param_file_arg,
        ekf_mode_arg,
        ekf_node_name_arg,
        use_sim_time_arg,
        pose_converter_node,
        ekf_localizer_node,
        initial_pose_publisher_node
    ])