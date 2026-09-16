#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gps_waypoint_logger — запись маршрута приёмником самого робота.

    ros2 run robot_navigation gps_waypoint_logger --ros-args -p output_file:=/home/pi/route.yaml
    # катаем робота пультом, в нужных местах:
    ros2 service call /gps_waypoint_logger/log_waypoint std_srvs/srv/Trigger
    # автоматически каждые N метров:
    ros2 run robot_navigation gps_waypoint_logger --ros-args -p auto_interval_m:=10.0

Берёт координаты из /gps/filtered (сглаженные EKF), а если их нет — из
/gps/fix. Курс — из /imu/data. Файл перезаписывается после каждой точки,
поэтому при обрыве ничего не теряется.
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import yaml

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_srvs.srv import Trigger


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * \
        math.sin(math.radians(lon2 - lon1) / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


class Logger(Node):
    def __init__(self):
        super().__init__('gps_waypoint_logger')
        self.declare_parameter('output_file', os.path.expanduser('~/route.yaml'))
        self.declare_parameter('auto_interval_m', 0.0)
        self.declare_parameter('save_yaw', False)
        self.out = str(self.get_parameter('output_file').value)
        self.auto_interval = float(self.get_parameter('auto_interval_m').value)
        self.save_yaw = bool(self.get_parameter('save_yaw').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.filtered = None
        self.raw = None
        self.yaw = None
        self.points = []
        self.create_subscription(NavSatFix, '/gps/filtered', self._on_filtered, qos)
        self.create_subscription(NavSatFix, '/gps/fix', self._on_raw, qos)
        self.create_subscription(Imu, '/imu/data', self._on_imu, qos)
        self.create_service(Trigger, '~/log_waypoint', self._log_srv)
        self.create_service(Trigger, '~/undo_last', self._undo_srv)
        if self.auto_interval > 0.0:
            self.create_timer(0.5, self._auto)
        self.get_logger().info(f'Пишу в {self.out}; вызовите ~/log_waypoint в нужных местах')

    def _on_filtered(self, m):
        self.filtered = m

    def _on_raw(self, m):
        if m.status.status >= NavSatStatus.STATUS_FIX:
            self.raw = m

    def _on_imu(self, m):
        q = m.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _current(self):
        fix = self.filtered or self.raw
        if fix is None or not math.isfinite(fix.latitude):
            return None
        return fix

    def _log(self):
        fix = self._current()
        if fix is None:
            return False, 'нет GPS-фикса'
        pt = {'latitude': round(fix.latitude, 7), 'longitude': round(fix.longitude, 7)}
        if self.save_yaw and self.yaw is not None:
            pt['yaw'] = round(self.yaw, 3)
        self.points.append(pt)
        self._write()
        src = 'filtered' if self.filtered else 'raw'
        msg = f'#{len(self.points)}: {pt["latitude"]}, {pt["longitude"]} ({src})'
        self.get_logger().info(msg)
        return True, msg

    def _write(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.out)), exist_ok=True)
        with open(self.out, 'w', encoding='utf-8') as f:
            yaml.safe_dump({'waypoints': self.points}, f, allow_unicode=True,
                           sort_keys=False)

    def _log_srv(self, _r, resp):
        resp.success, resp.message = self._log()
        return resp

    def _undo_srv(self, _r, resp):
        if self.points:
            self.points.pop()
            self._write()
            resp.success, resp.message = True, f'осталось {len(self.points)} точек'
        else:
            resp.success, resp.message = False, 'список пуст'
        return resp

    def _auto(self):
        fix = self._current()
        if fix is None:
            return
        if not self.points:
            self._log()
            return
        last = self.points[-1]
        if haversine_m(last['latitude'], last['longitude'],
                       fix.latitude, fix.longitude) >= self.auto_interval:
            self._log()


def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
