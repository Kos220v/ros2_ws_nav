#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
goto_point — поехать в ОДНУ указанную точку через Nav2 (NavigateToPose).

Точку можно задать тремя способами:

  1. GPS:            ros2 run robot_navigation goto_point -- --lat 56.2991 --lon 43.9231
     (WGS84 -> map через сервис /fromLL узла navsat_transform)

  2. В карте (map):  ros2 run robot_navigation goto_point -- --x 12.0 --y -3.5 [--yaw 1.57]

  3. Относительно робота (base_link — «5 м вперёд и 2 м влево»):
                     ros2 run robot_navigation goto_point -- --dx 5 --dy 2

Кроме CLI узел поднимает топики, чтобы точку можно было прислать из
приложения/rosbridge, не перезапуская ничего:

     /goto/gps      geographic_msgs/GeoPoint     — GPS-цель
     /goto/pose     geometry_msgs/PoseStamped    — цель в любом фрейме TF
     /goto/cancel   std_msgs/Empty               — отмена

Nav2 сам строит путь и объезжает препятствия по лидару. Команда скорости
уходит в /cmd_vel/auto, поэтому пульт в любой момент имеет приоритет.
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args

from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL
from std_msgs.msg import Empty, String

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped  # noqa: F401 (регистрация)


def yaw_to_q(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class GotoPoint(Node):
    def __init__(self, cli_args):
        super().__init__('goto_point')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('fromll_service', '/fromLL')
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.fromll = self.create_client(FromLL, str(self.get_parameter('fromll_service').value))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_handle = None
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(GeoPoint, '/goto/gps', self._on_gps_goal, 10)
        self.create_subscription(PoseStamped, '/goto/pose', self._on_pose_goal, 10)
        self.create_subscription(Empty, '/goto/cancel', lambda _m: self.cancel(), 10)

        self.cli = cli_args
        self.one_shot = cli_args is not None and any(
            v is not None for v in (cli_args.lat, cli_args.x, cli_args.dx))
        if self.one_shot:
            # Дать TF и сервисам подняться.
            self.create_timer(1.0, self._run_cli_once)
        self._cli_done = False

    # ------------------------------------------------------------- вход
    def _run_cli_once(self):
        if self._cli_done:
            return
        self._cli_done = True
        a = self.cli
        yaw = a.yaw if a.yaw is not None else None
        if a.lat is not None:
            self.goto_gps(a.lat, a.lon, yaw)
        elif a.x is not None:
            self.goto_map(a.x, a.y, yaw)
        else:
            self.goto_relative(a.dx, a.dy, yaw)

    def _on_gps_goal(self, msg: GeoPoint):
        self.goto_gps(msg.latitude, msg.longitude, None)

    def _on_pose_goal(self, msg: PoseStamped):
        if msg.header.frame_id in ('', self.global_frame):
            msg.header.frame_id = self.global_frame
            self._send(msg)
        else:
            self._send_in_frame(msg)

    # ------------------------------------------------------------- цели
    def goto_gps(self, lat, lon, yaw):
        self.get_logger().info(f'Цель GPS: {lat:.6f}, {lon:.6f}')
        if not self.fromll.wait_for_service(timeout_sec=10.0):
            self._fail('сервис /fromLL недоступен — navsat_transform не запущен '
                       'или нет GPS-фикса')
            return
        req = FromLL.Request()
        req.ll_point.latitude = float(lat)
        req.ll_point.longitude = float(lon)
        req.ll_point.altitude = 0.0
        fut = self.fromll.call_async(req)

        def done(f):
            res = f.result()
            if res is None:
                self._fail('ошибка вызова /fromLL')
                return
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = res.map_point.x
            pose.pose.position.y = res.map_point.y
            pose.pose.orientation = yaw_to_q(yaw if yaw is not None else 0.0)
            self.get_logger().info(
                f'GPS -> map: ({res.map_point.x:.1f}, {res.map_point.y:.1f})')
            self._send(pose)
        fut.add_done_callback(done)

    def goto_map(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = yaw_to_q(yaw if yaw is not None else 0.0)
        self._send(pose)

    def goto_relative(self, dx, dy, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = Time().to_msg()  # latest
        pose.pose.position.x = float(dx)
        pose.pose.position.y = float(dy or 0.0)
        pose.pose.orientation = yaw_to_q(yaw if yaw is not None else 0.0)
        self._send_in_frame(pose)

    def _send_in_frame(self, pose: PoseStamped):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, pose.header.frame_id, Time(),
                timeout=Duration(seconds=3.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self._fail(f'нет TF {self.global_frame} <- {pose.header.frame_id}: {e}')
            return
        goal = do_transform_pose_stamped(pose, tf)
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        self._send(goal)

    def _send(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self._fail('экшен navigate_to_pose недоступен — Nav2 не поднят?')
            return
        self.cancel(silent=True)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f'Едем в ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) '
            f'[{pose.header.frame_id}]')
        self._status('RUNNING')
        fut = self.nav_client.send_goal_async(goal, feedback_callback=self._fb)
        fut.add_done_callback(self._accepted)

    def _accepted(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._fail('Nav2 отклонил цель')
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self._result)

    def _fb(self, fb):
        d = fb.feedback.distance_remaining
        self.get_logger().info(f'Осталось {d:.1f} м', throttle_duration_sec=3.0)

    def _result(self, fut):
        st = fut.result().status
        self.goal_handle = None
        if st == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Цель достигнута')
            self._status('DONE')
        elif st == GoalStatus.STATUS_CANCELED:
            self._status('CANCELLED')
        else:
            self._fail(f'навигация не удалась, статус {st}')
        if self.one_shot:
            rclpy.shutdown()

    def cancel(self, silent=False):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            if not silent:
                self._status('CANCELLED')

    def _status(self, s):
        self.status_pub.publish(String(data=s))

    def _fail(self, msg):
        self.get_logger().error(msg)
        self._status('FAILED')
        if self.one_shot:
            rclpy.shutdown()


def parse_args(argv):
    ap = argparse.ArgumentParser(description='Поехать в точку через Nav2')
    ap.add_argument('--lat', type=float)
    ap.add_argument('--lon', type=float)
    ap.add_argument('--x', type=float, help='X в фрейме map, м')
    ap.add_argument('--y', type=float, help='Y в фрейме map, м')
    ap.add_argument('--dx', type=float, help='вперёд относительно робота, м')
    ap.add_argument('--dy', type=float, help='влево относительно робота, м')
    ap.add_argument('--yaw', type=float, help='желаемый курс в точке, рад (ENU)')
    a = ap.parse_args(argv)
    if a.lat is not None and a.lon is None:
        ap.error('--lat требует --lon')
    if a.x is not None and a.y is None:
        ap.error('--x требует --y')
    return a


def main(args=None):
    rclpy.init(args=args)
    argv = remove_ros_args(sys.argv)[1:]
    cli = parse_args(argv)
    node = GotoPoint(cli)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cancel(silent=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
