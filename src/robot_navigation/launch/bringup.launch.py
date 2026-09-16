#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
bringup.launch.py — запуск ВСЕГО стека автономного робота одной командой.

    # улица, маршрут по GPS
    ros2 launch robot_navigation bringup.launch.py \\
        declination_deg:=11.9 \\
        waypoints_file:=/home/pi/route.yaml

    # помещение / стенд, без GPS: автономный объезд и цели в метрах
    ros2 launch robot_navigation bringup.launch.py use_gps:=false

Порядок запуска задан таймерами:

    0 c   железо: VESC, IMU, robot_odom, GNSS, пульт, TF
          (гироскоп STM32 калибруется первые секунды — робот должен стоять)
    8 c   локализация: EKF (odom->base_link), EKF (map->odom), navsat_transform
   10 c   лидар (его мотор вибрирует, поэтому включается ПОСЛЕ калибровки;
          задержка задаётся аргументом lidar_delay)
   15 c   Nav2 (нужна готовая TF-цепочка map -> odom -> base_link)
   20 c   командир маршрута (ждёт экшен Nav2) и goto_point (цели по топикам)

Робот НЕ ПОЕДЕТ сразу после запуска. Командир маршрута стартует выключенным
и ждёт либо перевода тумблера пульта в режим AUTO, либо вызова сервиса
/gps_waypoint_commander/start_route.

ПЕРЕД ПЕРВЫМ ВЫЕЗДОМ обязательно прогоните проверку:
    ros2 run robot_navigation nav_preflight_check

ЗАМЕЧАНИЕ ПРО УСЛОВИЯ ЗАПУСКА
-----------------------------
Условия (condition=IfCondition(...)) навешены прямо на действия, а не через
GroupAction: GroupAction создаёт отдельную область видимости для аргументов
launch, и вложенные TimerAction с period-подстановкой падают с ошибкой
"launch configuration ... does not exist".
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = get_package_share_directory('robot_navigation')
    hw_share = get_package_share_directory('project_start')

    default_waypoints = os.path.join(nav_share, 'config', 'gps_waypoints.yaml')

    # ------------------------------------------------------------- аргументы
    args = [
        DeclareLaunchArgument(
            'declination_deg', default_value='11.9',
            description='Магнитное склонение в градусах (+ восточное). '
                        'Узнать: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml'),
        DeclareLaunchArgument(
            'imu_yaw_offset_deg', default_value='-48.0',
            description='Поправка угла монтажа IMU относительно оси X робота, '
                        'градусы (+ против часовой). Определяется heading_check'),
        DeclareLaunchArgument(
            'use_gps', default_value='true',
            description='true — уличная навигация по GNSS; false — без GPS '
                        '(map == odom, цели в метрах, автономный объезд)'),
        DeclareLaunchArgument(
            'waypoints_file', default_value=default_waypoints,
            description='YAML-файл с маршрутом из GPS-точек'),
        DeclareLaunchArgument(
            'number_of_loops', default_value='0',
            description='Сколько раз повторить маршрут (0 = один проезд)'),
        DeclareLaunchArgument(
            'use_hardware', default_value='true',
            description='Запускать слой железа (false — если он уже запущен)'),
        DeclareLaunchArgument(
            'use_navigation', default_value='true',
            description='Запускать Nav2 (false — только локализация)'),
        DeclareLaunchArgument(
            'use_commander', default_value='true',
            description='Запускать командира GPS-маршрута'),
        DeclareLaunchArgument(
            'lidar_delay', default_value='10.0',
            description='Задержка старта лидара, сек'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
            description='Файл параметров Nav2'),
    ]

    use_gps = LaunchConfiguration('use_gps')

    # ------------------------------------------------------- слой 1: железо
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_share, 'launch', 'start.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_hardware')),
        launch_arguments={
            'lidar_delay': LaunchConfiguration('lidar_delay'),
            'use_gps': use_gps,
            'declination_deg': LaunchConfiguration('declination_deg'),
            'imu_yaw_offset_deg': LaunchConfiguration('imu_yaw_offset_deg'),
            'odom_publish_tf': 'false',   # TF odom->base_link даёт EKF
            'odom_yaw_mode': 'absolute',
        }.items(),
    )

    # -------------------------------------------------- слой 2: локализация
    # Ждём 8 секунд: STM32 успевает откалибровать ноль гироскопа и выдать
    # устойчивый кватернион, драйверы — открыть порты.
    localization = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share, 'launch', 'localization.launch.py')),
                launch_arguments={
                    'use_gps': use_gps,
                    'start_odom': 'false',   # robot_odom уже в слое железа
                }.items(),
            ),
        ],
    )

    # ------------------------------------------------------ слой 3: Nav2
    navigation = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share, 'launch', 'navigation.launch.py')),
                condition=IfCondition(LaunchConfiguration('use_navigation')),
                launch_arguments={
                    'params_file': LaunchConfiguration('nav2_params_file'),
                }.items(),
            ),
        ],
    )

    # ------------------------------------------------ слой 4: командиры
    commander = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='robot_navigation',
                executable='gps_waypoint_commander',
                name='gps_waypoint_commander',
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_commander')),
                parameters=[{
                    'waypoints_file': LaunchConfiguration('waypoints_file'),
                    'number_of_loops': ParameterValue(
                        LaunchConfiguration('number_of_loops'), value_type=int),
                    # Никогда не стартуем сами: только по пульту или по сервису.
                    'autostart': False,
                    'use_rc_mode': True,
                    'require_gps_fix': ParameterValue(use_gps, value_type=bool),
                }],
            ),
            # Приём одиночных целей по топикам /goto/gps, /goto/pose
            Node(
                package='robot_navigation',
                executable='goto_point',
                name='goto_point',
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_navigation')),
            ),
        ],
    )

    return LaunchDescription(args + [
        hardware,
        localization,
        navigation,
        commander,
    ])
