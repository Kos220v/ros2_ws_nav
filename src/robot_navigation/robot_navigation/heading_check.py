#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
heading_check — сверка курса IMU с реальным направлением.

Печатает раз в секунду:
  * yaw из /imu/data (ENU: 0 = восток, +90 = север) и его пересчёт в
    компасный азимут (0 = север, по часовой);
  * /imu/azimuth — азимут, который считает прошивка STM32;
  * yaw из /odom (robot_odom, с учётом yaw_offset_deg) и /odometry/global (EKF).

Как пользоваться:
  1. Поставьте робота носом строго на СЕВЕР (по карте/ориентиру).
  2. Ожидаемый yaw ENU = 90°. Разница (90 - yaw_imu) — это поправка монтажа
     платы: впишите её в аргумент imu_yaw_offset_deg (start.launch.py /
     bringup.launch.py) и в imu_rpy URDF (tracked_robot.urdf.xacro).
  3. Повторите для востока (yaw = 0) — если ошибка та же, всё верно; если
     разная, магнитометр не откалиброван (см. README imu_stm32_bridge).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def enu_to_compass(yaw_deg):
    return (90.0 - yaw_deg) % 360.0


class HeadingCheck(Node):
    def __init__(self):
        super().__init__('heading_check')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.imu = self.az = self.odom = self.glob = None
        self.create_subscription(Imu, '/imu/data', lambda m: setattr(self, 'imu', m), qos)
        self.create_subscription(Float32, '/imu/azimuth', lambda m: setattr(self, 'az', m), qos)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), qos)
        self.create_subscription(Odometry, '/odometry/global',
                                 lambda m: setattr(self, 'glob', m), qos)
        self.create_timer(1.0, self._print)

    def _print(self):
        parts = []
        if self.imu is not None:
            y = yaw_of(self.imu.orientation)
            parts.append(f'IMU yaw={y:7.1f}° ENU (компас {enu_to_compass(y):5.1f}°)')
        else:
            parts.append('IMU: нет данных')
        if self.az is not None:
            parts.append(f'STM32 азимут={self.az.data:5.1f}°')
        if self.odom is not None:
            parts.append(f'/odom yaw={yaw_of(self.odom.pose.pose.orientation):7.1f}°')
        if self.glob is not None:
            parts.append(f'EKF map yaw={yaw_of(self.glob.pose.pose.orientation):7.1f}°')
        self.get_logger().info(' | '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = HeadingCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
