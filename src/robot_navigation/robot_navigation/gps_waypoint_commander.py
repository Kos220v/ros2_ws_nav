#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gps_waypoint_commander — проезд маршрута из GPS-точек через Nav2.

Читает YAML с точками (см. config/gps_waypoints.yaml), проверяет их и отдаёт
экшену Nav2 `/follow_gps_waypoints` (nav2_waypoint_follower). Сам Nav2
планирует путь, объезжает препятствия по лидару и ведёт робота.

Способы запуска маршрута:

  * тумблер пульта: /control_mode == 0 (AUTO)  -> старт маршрута
                    /control_mode == 1 (MANUAL) -> отмена
                    /control_mode == 3 (RETURN_HOME) -> возврат в точку старта
  * сервисы (std_srvs/Trigger):
        ~/start_route     начать (или перезапустить) маршрут
        ~/stop_route      отменить текущую задачу
        ~/return_home     ехать в точку, где был получен первый GPS-фикс
        ~/reload_route    перечитать YAML (например, после логгера)
  * параметр autostart: true — стартовать сразу (по умолчанию false).

Топики состояния:
        ~/status          std_msgs/String   IDLE | WAITING_GPS | RUNNING |
                                            RETURNING | DONE | FAILED | CANCELLED
        ~/current_waypoint std_msgs/Int32   индекс текущей точки
"""

import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import yaml

from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPose
from nav2_msgs.action import FollowGPSWaypoints
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Int8, Int32, String
from std_srvs.srv import Trigger


MODE_AUTO = 0
MODE_MANUAL = 1
MODE_AVOID = 2
MODE_RETURN_HOME = 3


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def yaw_to_quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def load_waypoints(path):
    """Возвращает список dict(lat, lon, yaw|None). Бросает ValueError."""
    if not path or not os.path.exists(path):
        raise ValueError(f'файл маршрута не найден: {path}')
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    raw = data.get('waypoints', data if isinstance(data, list) else [])
    if not isinstance(raw, list):
        raise ValueError("ожидался список 'waypoints'")
    out = []
    for i, wp in enumerate(raw):
        if not isinstance(wp, dict):
            raise ValueError(f'точка #{i}: ожидался словарь')
        lat = wp.get('latitude', wp.get('lat'))
        lon = wp.get('longitude', wp.get('lon'))
        if lat is None or lon is None:
            raise ValueError(f'точка #{i}: нет latitude/longitude')
        lat, lon = float(lat), float(lon)
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f'точка #{i}: координаты вне диапазона ({lat}, {lon})')
        yaw = wp.get('yaw')
        out.append({'lat': lat, 'lon': lon, 'yaw': None if yaw is None else float(yaw)})
    return out


class GpsWaypointCommander(Node):
    def __init__(self):
        super().__init__('gps_waypoint_commander')

        p = self.declare_parameter
        p('waypoints_file', '')
        p('number_of_loops', 0)
        p('autostart', False)
        p('use_rc_mode', True)
        p('require_gps_fix', True)
        p('gps_topic', '/gps/fix')
        p('control_mode_topic', '/control_mode')
        p('max_leg_distance', 55.0)     # м, ~половина скользящего global costmap
        p('min_leg_distance', 1.0)      # м, предупреждение о дублях
        p('max_first_leg_distance', 200.0)  # м, от робота до первой точки
        p('action_wait_timeout', 60.0)

        g = self.get_parameter
        self.waypoints_file = str(g('waypoints_file').value)
        self.number_of_loops = int(g('number_of_loops').value)
        self.autostart = bool(g('autostart').value)
        self.use_rc_mode = bool(g('use_rc_mode').value)
        self.require_gps_fix = bool(g('require_gps_fix').value)
        self.max_leg = float(g('max_leg_distance').value)
        self.min_leg = float(g('min_leg_distance').value)
        self.max_first_leg = float(g('max_first_leg_distance').value)
        self.action_wait_timeout = float(g('action_wait_timeout').value)

        self.waypoints = []
        self.last_fix = None
        self.home_fix = None
        self.current_mode = None
        self.goal_handle = None
        self.status = 'IDLE'
        self._pending_start = None   # 'route' | 'home' — ждём GPS-фикс
        self._loading_error = None

        self._reload()

        # ---------------------------------------------------------- интерфейсы
        self.client = ActionClient(self, FollowGPSWaypoints, 'follow_gps_waypoints')

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(NavSatFix, str(g('gps_topic').value), self._on_fix, sensor_qos)
        if self.use_rc_mode:
            self.create_subscription(Int8, str(g('control_mode_topic').value),
                                     self._on_mode, 10)

        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.wp_pub = self.create_publisher(Int32, '~/current_waypoint', 10)

        self.create_service(Trigger, '~/start_route', self._srv_start)
        self.create_service(Trigger, '~/stop_route', self._srv_stop)
        self.create_service(Trigger, '~/return_home', self._srv_home)
        self.create_service(Trigger, '~/reload_route', self._srv_reload)

        self.create_timer(1.0, self._tick)

        if self.autostart:
            self.get_logger().warning('autostart=true — маршрут стартует автоматически!')
            self._request('route')

        self.get_logger().info(
            f'Командир маршрута готов: {len(self.waypoints)} точек из '
            f'{self.waypoints_file or "<не задан>"}; ждёт AUTO на пульте '
            f'или сервис ~/start_route')

    # ------------------------------------------------------------- маршрут
    def _reload(self):
        try:
            self.waypoints = load_waypoints(self.waypoints_file)
            self._loading_error = None
            self._validate_route()
        except ValueError as e:
            self.waypoints = []
            self._loading_error = str(e)
            self.get_logger().error(f'Маршрут не загружен: {e}')

    def _validate_route(self):
        for i in range(1, len(self.waypoints)):
            a, b = self.waypoints[i - 1], self.waypoints[i]
            d = haversine_m(a['lat'], a['lon'], b['lat'], b['lon'])
            if d > self.max_leg:
                self.get_logger().warning(
                    f'Точки {i - 1}->{i}: {d:.0f} м — больше {self.max_leg:.0f} м, '
                    f'планировщик может не увидеть цель (добавьте промежуточные точки)')
            elif d < self.min_leg:
                self.get_logger().warning(
                    f'Точки {i - 1}->{i}: {d:.1f} м — почти дубли')

    def _fix_ok(self, fix):
        return (fix is not None and fix.status.status >= NavSatStatus.STATUS_FIX
                and math.isfinite(fix.latitude) and math.isfinite(fix.longitude)
                and not (fix.latitude == 0.0 and fix.longitude == 0.0))

    # ------------------------------------------------------------- callbacks
    def _on_fix(self, msg):
        if self._fix_ok(msg):
            self.last_fix = msg
            if self.home_fix is None:
                self.home_fix = msg
                self.get_logger().info(
                    f'Точка «дом» зафиксирована: {msg.latitude:.6f}, {msg.longitude:.6f}')

    def _on_mode(self, msg):
        mode = int(msg.data)
        if mode == self.current_mode:
            return
        prev = self.current_mode
        self.current_mode = mode
        if prev is None and mode != MODE_AUTO and mode != MODE_RETURN_HOME:
            return
        if mode == MODE_AUTO:
            self.get_logger().info('Пульт: AUTO -> старт маршрута')
            self._request('route')
        elif mode == MODE_RETURN_HOME:
            self.get_logger().info('Пульт: RETURN_HOME -> возврат домой')
            self._request('home')
        else:
            self.get_logger().info('Пульт: MANUAL -> отмена автономной задачи')
            self._cancel()

    # ------------------------------------------------------------- сервисы
    def _srv_start(self, _req, resp):
        if not self.waypoints:
            resp.success = False
            resp.message = f'маршрут пуст: {self._loading_error or self.waypoints_file}'
            return resp
        self._request('route')
        resp.success = True
        resp.message = f'маршрут из {len(self.waypoints)} точек запущен'
        return resp

    def _srv_stop(self, _req, resp):
        self._cancel()
        resp.success = True
        resp.message = 'задача отменена'
        return resp

    def _srv_home(self, _req, resp):
        if self.home_fix is None:
            resp.success = False
            resp.message = 'точка «дом» ещё не известна (нет GPS-фикса с момента старта)'
            return resp
        self._request('home')
        resp.success = True
        resp.message = 'едем домой'
        return resp

    def _srv_reload(self, _req, resp):
        self._reload()
        resp.success = self._loading_error is None
        resp.message = (f'загружено {len(self.waypoints)} точек' if resp.success
                        else self._loading_error)
        return resp

    # ------------------------------------------------------------- логика
    def _request(self, what):
        if self.require_gps_fix and not self._fix_ok(self.last_fix):
            self._pending_start = what
            self._set_status('WAITING_GPS')
            self.get_logger().warning('Нет валидного GPS-фикса — жду...')
            return
        self._pending_start = None
        self._cancel(silent=True)
        if what == 'home':
            self._send([{'lat': self.home_fix.latitude, 'lon': self.home_fix.longitude,
                         'yaw': None}], loops=0, status='RETURNING')
        else:
            if not self.waypoints:
                self.get_logger().error('Маршрут пуст — ехать некуда')
                self._set_status('FAILED')
                return
            if self.last_fix is not None:
                w0 = self.waypoints[0]
                d = haversine_m(self.last_fix.latitude, self.last_fix.longitude,
                                w0['lat'], w0['lon'])
                if d > self.max_first_leg:
                    self.get_logger().error(
                        f'До первой точки {d:.0f} м (> {self.max_first_leg:.0f}). '
                        'Похоже, маршрут не для этого места. Отказ.')
                    self._set_status('FAILED')
                    return
            self._send(self.waypoints, loops=self.number_of_loops, status='RUNNING')

    def _tick(self):
        if self._pending_start and self._fix_ok(self.last_fix):
            what = self._pending_start
            self._pending_start = None
            self._request(what)
        self.status_pub.publish(String(data=self.status))

    def _set_status(self, s):
        if s != self.status:
            self.get_logger().info(f'Статус: {self.status} -> {s}')
        self.status = s
        self.status_pub.publish(String(data=s))

    def _send(self, points, loops, status):
        if not self.client.wait_for_server(timeout_sec=self.action_wait_timeout):
            self.get_logger().error('Экшен /follow_gps_waypoints недоступен — Nav2 не поднят?')
            self._set_status('FAILED')
            return
        goal = FollowGPSWaypoints.Goal()
        goal.number_of_loops = int(loops)
        goal.goal_index = 0
        for wp in points:
            gp = GeoPose()
            gp.position.latitude = wp['lat']
            gp.position.longitude = wp['lon']
            gp.position.altitude = 0.0
            gp.orientation = yaw_to_quaternion(wp['yaw'] or 0.0)
            goal.gps_poses.append(gp)
        self._set_status(status)
        fut = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Nav2 отклонил задачу')
            self._set_status('FAILED')
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, fb):
        idx = int(fb.feedback.current_waypoint)
        self.wp_pub.publish(Int32(data=idx))
        self.get_logger().info(f'Точка {idx + 1}', throttle_duration_sec=5.0)

    def _on_result(self, fut):
        res = fut.result()
        self.goal_handle = None
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            missed = list(res.result.missed_waypoints)
            if missed:
                self.get_logger().warning(
                    f'Маршрут завершён, пропущены точки: {[m.index for m in missed]}')
            else:
                self.get_logger().info('Маршрут успешно завершён')
            self._set_status('DONE')
        elif res.status == GoalStatus.STATUS_CANCELED:
            self._set_status('CANCELLED')
        else:
            self.get_logger().error(f'Маршрут прерван, статус {res.status}')
            self._set_status('FAILED')

    def _cancel(self, silent=False):
        self._pending_start = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            if not silent:
                self._set_status('CANCELLED')
        elif not silent:
            self._set_status('IDLE')


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel(silent=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
