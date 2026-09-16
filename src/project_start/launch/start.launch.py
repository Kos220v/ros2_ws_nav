#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
start.launch.py — слой ЖЕЛЕЗА гусеничного робота (Raspberry Pi 4, Ubuntu 24.04,
ROS 2 Jazzy).

Запускает:
    elrs_receiver        пульт ELRS       -> /cmd_vel/manual, /control_mode
    kolesa_control       2×VESC (FS75100) <- /cmd_vel, -> /odom/vesc (скорость)
    imu_stm32_bridge     STM32 IMU        -> /imu/data (кватернион ENU, гироскоп)
    robot_odom           /odom/vesc + /imu/data -> /odom
                         (путь — VESC, курс — IMU; колёсный yaw НЕ используется)
    nmea_navsat_driver   GNSS             -> /gps/fix
    robot_state_publisher  URDF           -> статические TF base_link -> датчики
    cmd_switcher         приоритеты       -> /cmd_vel
    relay_reliable       /scan -> /scan_reliable
    ydlidar              лидар            -> /scan (с задержкой lidar_delay)

Аргументы:
    lidar_delay   задержка старта лидара, с (мотор вибрирует, IMU должна
                  успеть откалибровать гироскоп стоя)
    use_gps       запускать драйвер GNSS (false — стенд/помещение)
    gps_port, imu_port, lidar_port   переопределение устройств
    odom_publish_tf   true — robot_odom сам публикует TF odom->base_link
                      (ТОЛЬКО без robot_localization, т.е. без localization.launch.py)
    odom_yaw_mode     absolute (ENU, нужно для GPS) | relative (ноль при старте)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
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
    odom_share = get_package_share_directory('robot_odom')

    lidar_port = LaunchConfiguration('lidar_port').perform(context) or \
        _first_existing('/dev/lidar', '/dev/ttyUSB0')
    gps_port = LaunchConfiguration('gps_port').perform(context) or \
        _first_existing('/dev/gps', '/dev/ttyUSB1', '/dev/ttyACM0')
    imu_port = LaunchConfiguration('imu_port').perform(context) or \
        _first_existing('/dev/imu_stm32', '/dev/ttyUSB2')
    use_gps = LaunchConfiguration('use_gps').perform(context).lower() in ('1', 'true', 'yes')
    lidar_delay = float(LaunchConfiguration('lidar_delay').perform(context))

    if lidar_port is None:
        raise RuntimeError('Лидар не найден: нет /dev/lidar и /dev/ttyUSB0 '
                           '(задайте lidar_port:=...)')
    if imu_port is None:
        raise RuntimeError('IMU не найден: нет /dev/imu_stm32 и /dev/ttyUSB2 '
                           '(задайте imu_port:=...)')
    if use_gps and gps_port is None:
        raise RuntimeError('GNSS не найден: нет /dev/gps, /dev/ttyUSB1, /dev/ttyACM0 '
                           '(задайте gps_port:=... или use_gps:=false)')

    # ------------------------------------------------------------ пульт ELRS
    elrs_node = Node(
        package='elrs_receiver',
        executable='elrs_node',
        name='elrs_receiver',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
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
    gps_nodes = []
    if use_gps:
        gps_nodes.append(Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_navsat_driver',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                'port': gps_port,
                'baud': int(LaunchConfiguration('gps_baud').perform(context)),
                'frame_id': 'gps_link',
                'useRMC': False,
            }],
            remappings=[
                ('fix', 'gps/fix'),
                ('vel', 'gps/vel'),
                ('time_reference', 'gps/time_reference'),
                ('heading', 'gps/heading'),
            ],
        ))

    # --------------------------------------------------------------- приводы
    kolesa_control_node = Node(
        package='kolesa_control',
        executable='kolesa_control',
        name='kolesa_control',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'left_port': '/dev/ttyAMA4',
            'right_port': '/dev/ttyAMA5',
            'baud': 115200,
            'wheel_separation': 0.48,
            # Калибровка одометрии VESC (см. kolesa_control/README.md)
            'tacho_counts_per_revolution': 2157.0,
            'distance_per_revolution': 2.011,
            'odometry_scale': 1.0,
            'invert_left': False,
            'invert_right': True,
            'encoder_invert_left': False,
            'encoder_invert_right': True,
            'invert_angular': False,
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 1.0,
            'duty_min': 0.03,
            'duty_max': 0.6,
            'publish_odom': True,
            'odom_topic': 'odom/vesc',
            'publish_joint_states': True,
            'publish_diagnostics': True,
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
                # Магнитное склонение, градусы (+ восточное). Пересчитайте для
                # своей местности: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
                'declination': ParameterValue(
                    LaunchConfiguration('declination_deg'), value_type=float),
                'publish_mag': True,
            },
        ],
    )

    # ---------------------------------------------- одометрия VESC + IMU -> /odom
    odom_params = os.path.join(odom_share, 'config', 'odom_params.yaml')
    odom_node = Node(
        package='robot_odom',
        executable='odom_node',
        name='robot_odom',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[odom_params, {
            'vesc_odom_topic': '/odom/vesc',
            'imu_topic': '/imu/data',
            'odom_topic': '/odom',
            'publish_tf': ParameterValue(
                LaunchConfiguration('odom_publish_tf'), value_type=bool),
            'yaw_mode': LaunchConfiguration('odom_yaw_mode'),
            'yaw_offset_deg': ParameterValue(
                LaunchConfiguration('imu_yaw_offset_deg'), value_type=float),
        }],
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
        respawn=True,
        respawn_delay=2.0,
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
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            ydlidar_params,
            {'port': lidar_port},
        ],
    )
    lidar_delayed = TimerAction(period=lidar_delay, actions=[ydlidar_node])

    # /scan: RELIABLE-копия для Nav2 (костмапы), Wi-Fi и rosbridge
    relay_node = Node(
        package='relay_reliable',
        executable='relay_node',
        name='relay_reliable',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return [
        elrs_node,
        imu_node,
        kolesa_control_node,
        odom_node,
        *gps_nodes,
        robot_state_publisher_node,
        cmd_mux_node,
        relay_node,
        lidar_delayed,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_delay', default_value='10.0',
                              description='Задержка старта лидара, с'),
        DeclareLaunchArgument('use_gps', default_value='true',
                              description='Запускать драйвер GNSS'),
        DeclareLaunchArgument('gps_port', default_value='',
                              description='Порт GNSS (пусто = автопоиск)'),
        DeclareLaunchArgument('gps_baud', default_value='115200'),
        DeclareLaunchArgument('imu_port', default_value='',
                              description='Порт IMU STM32 (пусто = автопоиск)'),
        DeclareLaunchArgument('lidar_port', default_value='',
                              description='Порт лидара (пусто = автопоиск)'),
        DeclareLaunchArgument('declination_deg', default_value='11.9',
                              description='Магнитное склонение, град (+ восточное)'),
        DeclareLaunchArgument('imu_yaw_offset_deg', default_value='-48.0',
                              description='Поправка угла монтажа IMU, град'),
        DeclareLaunchArgument('odom_publish_tf', default_value='false',
                              description='robot_odom публикует TF odom->base_link '
                                          '(true только БЕЗ robot_localization)'),
        DeclareLaunchArgument('odom_yaw_mode', default_value='absolute',
                              description='absolute (ENU) | relative (ноль при старте)'),
        OpaqueFunction(function=launch_setup),
    ])
