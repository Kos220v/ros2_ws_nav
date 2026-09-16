#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
localization.launch.py — оценка положения робота на местности.

Источники:

    kolesa_control      -> /odom/vesc  (скорость по тахометрам VESC)
    imu_stm32_bridge    -> /imu/data   (кватернион ENU, гироскоп; склонение
                                        и калибровка компаса — в прошивке STM32)
    nmea_navsat_driver  -> /gps/fix

Цепочка:

    /odom/vesc + /imu/data -> robot_odom -> /odom  (X/Y интеграл скорости VESC
                                                    по курсу IMU; yaw = IMU)
    /odom + /imu/data            -> ekf_filter_node_odom -> TF odom -> base_link
                                                            /odometry/local
    /odom + /imu/data + /odometry/gps -> ekf_filter_node_map -> TF map -> odom
                                                                /odometry/global
    /gps/fix + /imu/data + /odometry/global -> navsat_transform -> /odometry/gps
                                                                   сервис /fromLL

Внешний AHRS (imu_filter_madgwick) больше не нужен: ориентацию считает
прошивка STM32 (MPU6050 + QMC5883L, 9-осевое слияние) и отдаёт готовый
кватернион корпус->ENU уже с учётом магнитного склонения (параметр
`declination` у imu_stm32_bridge).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation')
    odom_share = get_package_share_directory('robot_odom')
    config_dir = os.path.join(pkg_share, 'config')

    ekf_params = os.path.join(config_dir, 'dual_ekf_navsat.yaml')
    odom_params = os.path.join(odom_share, 'config', 'odom_params.yaml')

    start_odom_arg = DeclareLaunchArgument(
        'start_odom', default_value='false',
        description='Запустить robot_odom здесь (true, если слой железа '
                    'project_start/start.launch.py не запущен)',
    )
    use_gps_arg = DeclareLaunchArgument(
        'use_gps', default_value='true',
        description='true — полная схема (два EKF + navsat_transform). '
                    'false — без GNSS: только локальный EKF, map == odom '
                    '(помещение/стенд: автономный объезд и цели в метрах)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Использовать /clock вместо системного времени',
    )
    imu_yaw_offset_arg = DeclareLaunchArgument(
        'imu_yaw_offset_deg', default_value='0.0',
        description='Поправка угла монтажа IMU относительно оси X робота, '
                    'градусы, + против часовой (см. heading_check)',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gps = LaunchConfiguration('use_gps')

    # ------------------------------------------------ одометрия VESC + IMU
    robot_odom_node = Node(
        package='robot_odom',
        executable='odom_node',
        name='robot_odom',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('start_odom')),
        parameters=[odom_params, {
            'publish_tf': False,          # TF odom->base_link даёт EKF
            'yaw_mode': 'absolute',       # курс ENU — нужен navsat_transform
            'yaw_offset_deg': ParameterValue(
                LaunchConfiguration('imu_yaw_offset_deg'), value_type=float),
            'use_sim_time': use_sim_time,
        }],
    )

    # -------------------------------------------------------- локальный EKF
    ekf_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/local'),
            ('accel/filtered', 'accel/local'),
            ('/set_pose', '/set_pose_local'),
        ],
    )

    # -------------------------------------------------------- глобальный EKF
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(use_gps),
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/global'),
            ('accel/filtered', 'accel/global'),
            ('/set_pose', '/set_pose_global'),
        ],
    )

    # ------------------------------------------------------ WGS84 -> метры
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(use_gps),
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu', 'imu/data'),
            ('gps/fix', 'gps/fix'),
            ('odometry/filtered', 'odometry/global'),
            ('odometry/gps', 'odometry/gps'),
            ('gps/filtered', 'gps/filtered'),
        ],
    )

    return LaunchDescription([
        start_odom_arg,
        use_sim_time_arg,
        imu_yaw_offset_arg,
        robot_odom_node,
        ekf_odom_node,
        ekf_map_node,
        navsat_transform_node,
    ])
