"""Запуск одометрии VESC+IMU отдельно (для стенда)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_odom')
    params = os.path.join(pkg, 'config', 'odom_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Публиковать TF odom->base_link '
                                          '(false, если работает EKF)'),
        DeclareLaunchArgument('yaw_mode', default_value='relative',
                              description='absolute (ENU с IMU) | relative (ноль при старте)'),
        Node(
            package='robot_odom',
            executable='odom_node',
            name='robot_odom',
            output='screen',
            parameters=[params, {
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                'yaw_mode': LaunchConfiguration('yaw_mode'),
            }],
        ),
    ])
