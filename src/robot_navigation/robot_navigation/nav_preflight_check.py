#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
nav_preflight_check — проверка готовности стека перед выездом.

    ros2 run robot_navigation nav_preflight_check

Проверяет за ~6 секунд:
  * частоты топиков: /odom/vesc, /imu/data, /odom, /gps/fix, /scan_reliable,
    /odometry/local, /odometry/global;
  * что IMU отдаёт ориентацию (кватернион не нулевой);
  * что GPS-фикс валиден;
  * TF-цепочку map -> odom -> base_link -> laser_frame / imu_link / gps_link;
  * что живы экшены Nav2 (navigate_to_pose, follow_gps_waypoints) и /fromLL.

В конце — сводка OK / WARN / FAIL. Код возврата 1, если есть FAIL.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus
import tf2_ros

DURATION = 6.0

TOPICS = [
    # имя, тип, минимальная частота, обязателен
    ('/odom/vesc', Odometry, 10.0, True),
    ('/imu/data', Imu, 20.0, True),
    ('/odom', Odometry, 10.0, True),
    ('/gps/fix', NavSatFix, 0.5, True),
    ('/scan_reliable', LaserScan, 4.0, True),
    ('/odometry/local', Odometry, 10.0, True),
    ('/odometry/global', Odometry, 10.0, True),
]

TF_PAIRS = [
    ('map', 'odom', True),
    ('odom', 'base_link', True),
    ('base_link', 'laser_frame', True),
    ('base_link', 'imu_link', True),
    ('base_link', 'gps_link', False),
]


class Preflight(Node):
    def __init__(self):
        super().__init__('nav_preflight_check')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.counts = {t[0]: 0 for t in TOPICS}
        self.last = {}
        for name, typ, _, _ in TOPICS:
            self.create_subscription(typ, name, self._mk_cb(name), qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.results = []

    def _mk_cb(self, name):
        def cb(msg):
            self.counts[name] += 1
            self.last[name] = msg
        return cb

    def report(self, level, text):
        self.results.append((level, text))
        # rclpy запрещает менять severity у одного и того же места вызова,
        # поэтому три отдельных вызова, а не один через словарь.
        if level == 'OK':
            self.get_logger().info(f'[OK] {text}')
        elif level == 'WARN':
            self.get_logger().warning(f'[WARN] {text}')
        else:
            self.get_logger().error(f'[FAIL] {text}')

    def evaluate(self):
        for name, _, min_hz, required in TOPICS:
            hz = self.counts[name] / DURATION
            if hz >= min_hz:
                self.report('OK', f'{name}: {hz:.1f} Гц')
            elif hz > 0:
                self.report('WARN', f'{name}: {hz:.1f} Гц (ожидалось >= {min_hz})')
            else:
                self.report('FAIL' if required else 'WARN', f'{name}: нет сообщений')

        imu = self.last.get('/imu/data')
        if imu is not None:
            q = imu.orientation
            n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
            if n < 0.5 or imu.orientation_covariance[0] < 0:
                self.report('FAIL', 'IMU не отдаёт ориентацию (калибровка? 6x режим?)')
            else:
                self.report('OK', 'IMU отдаёт кватернион')

        fix = self.last.get('/gps/fix')
        if fix is not None:
            if fix.status.status < NavSatStatus.STATUS_FIX:
                self.report('FAIL', 'GPS без фикса')
            else:
                cov = fix.position_covariance[0] ** 0.5 if fix.position_covariance[0] > 0 else 0
                lvl = 'OK' if cov < 5.0 else 'WARN'
                self.report(lvl, f'GPS фикс {fix.latitude:.6f},{fix.longitude:.6f} '
                                 f'(σ≈{cov:.1f} м)')

        scan = self.last.get('/scan_reliable')
        if scan is not None:
            finite = sum(1 for r in scan.ranges if r == r and r != float('inf'))
            self.report('OK' if finite > 0 else 'WARN',
                        f'лидар: {len(scan.ranges)} лучей, {finite} с дальностью, '
                        f'frame={scan.header.frame_id}')

        for parent, child, required in TF_PAIRS:
            try:
                self.tf_buffer.lookup_transform(parent, child, Time())
                self.report('OK', f'TF {parent} -> {child}')
            except Exception as e:  # noqa: BLE001
                self.report('FAIL' if required else 'WARN',
                            f'TF {parent} -> {child}: {type(e).__name__}')

        names = {n for n, _ in self.get_service_names_and_types()}
        for srv in ('/fromLL', '/navigate_to_pose/_action/send_goal',
                    '/follow_gps_waypoints/_action/send_goal'):
            if srv in names:
                self.report('OK', f'сервис {srv}')
            else:
                self.report('FAIL', f'нет {srv} (Nav2/navsat_transform не подняты?)')

        fails = sum(1 for lvl, _ in self.results if lvl == 'FAIL')
        warns = sum(1 for lvl, _ in self.results if lvl == 'WARN')
        self.get_logger().info('=' * 60)
        if fails:
            self.get_logger().error(f'ИТОГ: {fails} FAIL, {warns} WARN — выезд НЕ рекомендован')
        elif warns:
            self.get_logger().warning(f'ИТОГ: {warns} WARN — проверьте замечания')
        else:
            self.get_logger().info('ИТОГ: всё в норме, можно ехать')
        return fails == 0


def main(args=None):
    rclpy.init(args=args)
    node = Preflight()
    node.get_logger().info(f'Собираю данные {DURATION:.0f} с...')
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < DURATION:
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = node.evaluate()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
