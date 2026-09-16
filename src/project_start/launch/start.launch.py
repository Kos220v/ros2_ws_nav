#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _first_existing(*paths):
    """Возвращает первый существующий путь или None."""
    for p in paths:
        if p and os.path.exists(p):
            return p
    return None


def launch_setup(context, *args, **kwargs):
    project_start_share = get_package_share_directory('project_start')
    imu_share = get_package_share_directory('imu_stm32_bridge')

    lidar_port = _first_existing('/dev/lidar', '/dev/ttyUSB0')
    gps_port = _first_existing('/dev/gps', '/dev/ttyUSB1')
    imu_port = _first_existing('/dev/imu_stm32', '/dev/ttyUSB2')

    if lidar_port is None:
        raise RuntimeError('Лидар не найден: нет /dev/lidar и /dev/ttyUSB0')
    if imu_port is None:
        raise RuntimeError('IMU не найден: нет /dev/imu_stm32 и /dev/ttyUSB2')

    # ------------------------------------------------------------ пульт ELRS
    elrs_node = Node(
        package='elrs_receiver',
        executable='elrs_node',
        name='elrs_receiver',
        output='screen',
        parameters=[{
            'port': '/dev/ttyAMA2',
            'baudrate': 420000,
            'deadzone': 0.02,
            'throttle_channel': 1,
            'steering_channel': 0,
            'invert_throttle': False,
            'invert_steering': False,
            'channel_min': 172,
            'channel_center': 992,
            'channel_max': 1811,
        }],
    )

    # ------------------------------------------------------------------ GNSS
    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_navsat_driver',
        output='screen',
        parameters=[{
            'port': gps_port,
            'baud': 115200,
            'frame_id': 'gps_link',
            'useRMC': False,
        }],
        remappings=[
            ('fix', 'gps/fix'),
        ],
    )

    # --------------------------------------------------------------- приводы
    kolesa_control_node = Node(
        package='kolesa_control',
        executable='kolesa_control',
        name='kolesa_control',
        output='screen',
        parameters=[{
            'left_port': '/dev/ttyAMA4',
            'right_port': '/dev/ttyAMA5',
            'baud': 115200,
            'wheel_radius': 0.0723,
            'wheel_separation': 0.48,
            'gear_ratio': 19.5,
            'pole_pairs': 2,
            'invert_left': False,
            'invert_right': True,
            'encoder_invert_left': False,
            'encoder_invert_right': True,
            'left_wheel_joint': 'left_track_joint',
            'right_wheel_joint': 'right_track_joint',
            'invert_angular': False,
        }],
    )

    # ------------------------------------------------- колёсная одометрия
    odom_node = Node(
        package='robot_odom',
        executable='odom_node',
        name='robot_odom',
        output='screen',
        parameters=[{
            'publish_tf': False,
        }],
    )

    # ------------------------------------------------------- инерциальный модуль
    imu_params = os.path.join(imu_share, 'config', 'imu_params.yaml')
    imu_node = Node(
        package='imu_stm32_bridge',
        executable='bridge_node',
        name='imu_stm32_bridge',
        namespace='imu',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            imu_params,
            {
                'port': imu_port,
                'baud': 115200,
                'frame_id': 'imu_link',
                'rate': 50,
                'declination': 13.43,
                'publish_mag': True,
            },
        ],
    )

    # ------------------------------------------------------------- TF из URDF
    urdf_file = os.path.join(
        get_package_share_directory('tracked_robot_description'),
        'urdf', 'tracked_robot.urdf.xacro'
    )
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ------------------------------------------------- приоритеты команд
    cmd_mux_node = Node(
        package='cmd_switcher',
        executable='cmd_mux_node',
        name='cmd_switcher',
        output='screen',
    )

    # ------------------------------------------------------------------ лидар
    ydlidar_params = os.path.join(project_start_share, 'params',
                                  'ydlidar_params.yaml')

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            ydlidar_params,
            {'port': lidar_port},
        ],
    )

    # /scan: RELIABLE-копия для Wi-Fi и rosbridge
    relay_node = Node(
        package='relay_reliable',
        executable='relay_node',
        name='relay_reliable',
        output='screen',
    )

    return [
        elrs_node,
        imu_node,
        # gps_node,
        kolesa_control_node,
        # odom_node,
        robot_state_publisher_node,
        cmd_mux_node,
        relay_node,
        ydlidar_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])