"""Only the Scout data-collection drivers; no Autoware planning or control."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sensor_share = get_package_share_directory('scout_sensor_kit_launch')
    vehicle_share = get_package_share_directory('scout_vehicle_description')
    zed_share = get_package_share_directory('zed_wrapper')
    zed_config = os.environ['SCOUT_DATASET_ZED_CONFIG']
    repository = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can10'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        Node(
            package='scout_base', executable='scout_base_node',
            name='scout_base', output='screen',
            parameters=[{
                'port_name': LaunchConfiguration('can_interface'),
                'auto_reconnect': False,
                'read_only': True,
                'odom_frame': 'scout_wheel_odom',
                'base_frame': 'scout_chassis_link',
                'odom_topic_name': '/scout/wheel_odometry',
            }],
            remappings=[
                ('/cmd_vel', '/scout/cmd_vel'),
                ('/scout_status', '/scout/status'),
                ('/scout_rc_state', '/scout/rc_state'),
                ('/tf', '/scout/wheel_tf'),
            ],
        ),
        ExecuteProcess(
            cmd=[sys.executable,
                 os.path.join(repository, 'scripts', 'scout_rc_command_estimator.py'),
                 '--ros-args', '--params-file',
                 os.path.join(repository, 'config', 'scout_rc_command.yaml')],
            output='screen',
        ),
        Node(
            package='fixposition_driver_ros2', executable='fixposition_driver_ros2_exec',
            name='fixposition_driver', output='screen',
            parameters=[os.path.join(sensor_share, 'config', 'fixposition.yaml'),
                        {'nav2_mode': False}],
            remappings=[
                ('/fixposition/poiimu', '/sensing/imu/imu_data'),
                ('/tf', '/fixposition/tf'),
                ('/tf_static', '/fixposition/tf_static'),
            ],
        ),
        Node(
            package='rslidar_sdk', executable='rslidar_sdk_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_lidar')),
            parameters=[{'config_path': os.path.join(sensor_share, 'config', 'rslidar.yaml')}],
            remappings=[('param_handle:__node', 'rslidar_sdk')],
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='scout_dataset_state_publisher', output='screen',
            parameters=[{'robot_description': ParameterValue(Command([
                'xacro ', os.path.join(vehicle_share, 'urdf', 'vehicle.xacro')
            ]), value_type=str)}],
        ),
        # vehicle.xacro defines the ZED mount; the official wrapper defines the
        # internal camera/optical frames. Join the two static frame trees.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='scout_zed_mount_tf', output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'zed2i_link', '--child-frame-id', 'zed2i_camera_link',
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(zed_share, 'launch', 'zed_camera.launch.py')),
            launch_arguments={
                'camera_model': 'zed2i',
                'camera_name': 'zed2i',
                'ros_params_override_path': zed_config,
                'publish_tf': 'false',
                'publish_map_tf': 'false',
                'publish_imu_tf': 'true',
            }.items(),
        ),
    ])
