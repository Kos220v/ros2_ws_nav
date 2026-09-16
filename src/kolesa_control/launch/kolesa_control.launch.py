# -*- coding: utf-8 -*-
"""Запуск ноды kolesa_control с параметрами.

Отредактируйте значения под свой робот и запустите:
    ros2 launch kolesa_control kolesa_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kolesa_control",
            executable="kolesa_control",
            name="kolesa_control",
            output="screen",
            parameters=[{
                # Порты (Raspberry Pi: uart4/uart5)
                "left_port": "/dev/ttyAMA4",
                "right_port": "/dev/ttyAMA5",
                "baud": 115200,

                # Колея, м — только для раскладки cmd_vel по бортам
                "wheel_separation": 0.48,

                # Калибровка одометрии VESC (README.md, «Калибровка одометрии»)
                "tacho_counts_per_revolution": 2157.0,
                "distance_per_revolution": 2.011,
                "odometry_scale": 1.0,

                # Направления
                "invert_left": False,
                "invert_right": True,
                "encoder_invert_left": False,
                "encoder_invert_right": True,
                "invert_angular": False,

                # Скважность VESC (управление разомкнутое — см. README.md)
                "duty_min": 0.03,
                "duty_max": 0.6,
                "max_linear_velocity": 1.0,
                "max_angular_velocity": 1.0,

                # Тайминги
                "control_rate": 50.0,
                "telemetry_rate": 20.0,
                "cmd_timeout": 0.5,

                # Публикации
                "publish_odom": True,          # /odom/vesc — скорость для robot_odom
                "odom_topic": "odom/vesc",
                "publish_joint_states": True,
                "publish_diagnostics": True,
            }],
        ),
    ])
