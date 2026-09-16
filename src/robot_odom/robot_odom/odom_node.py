#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
robot_odom/odom_node — одометрия «VESC + IMU».

Источники (и ТОЛЬКО они):

    /odom/vesc  nav_msgs/Odometry   от kolesa_control: twist.linear.x —
                                    линейная скорость центра робота по
                                    тахометрам VESC.
    /imu/data   sensor_msgs/Imu     от imu_stm32_bridge: orientation —
                                    кватернион корпус->ENU (абсолютный курс),
                                    angular_velocity.z — скорость рыскания.

Угол поворота по разности бортов НЕ вычисляется. Гусеничная машина в
повороте буксует, поэтому «колёсный» yaw бесполезен; курс берётся с IMU.

Выход:

    /odom       nav_msgs/Odometry   pose.x/y — интеграл v·cos(yaw), v·sin(yaw)
                                    pose.yaw — курс из КВАТЕРНИОНА IMU
                                    twist.linear.x — скорость VESC
                                    twist.angular.z — гироскоп IMU (справочно,
                                    в курс НЕ интегрируется ни здесь, ни в EKF)
    TF odom -> base_link            только если publish_tf: true
                                    (в полном стеке TF публикует EKF).

Сервисы:

    ~/reset_odom  std_srvs/Trigger  обнулить X, Y (и при yaw_mode=relative —
                                    заново взять текущий курс за ноль).

Режимы курса (yaw_mode):

    absolute  — yaw = курс IMU в ENU (0 = восток, +90° = север). Это то, что
                нужно для robot_localization + navsat_transform: локальная
                одометрия и GPS живут в одной системе отсчёта.
    relative  — курс обнуляется в момент старта/сброса. Удобно для стенда
                и езды в помещении без GPS.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomNode(Node):
    def __init__(self):
        super().__init__('robot_odom')

        p = self.declare_parameter
        p('vesc_odom_topic', '/odom/vesc')
        p('imu_topic', '/imu/data')
        p('odom_topic', '/odom')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('publish_tf', False)
        p('publish_rate', 0.0)          # 0 = публиковать на каждое сообщение VESC
        p('yaw_mode', 'absolute')       # absolute | relative
        p('yaw_offset_deg', 0.0)        # поправка монтажа IMU (+ против часовой)
        p('invert_yaw', False)          # если IMU смонтирован «вверх ногами»
        p('imu_timeout', 0.5)           # с; при просрочке интеграция замораживается
        p('vesc_timeout', 0.5)
        p('min_speed_for_integration', 0.0)  # м/с; отсечь дрожание нуля
        p('pose_xy_stddev_per_m', 0.05)      # рост неопределённости позиции на метр пути
        p('yaw_stddev', 0.03)                # рад (~1.7°) — доверие к курсу IMU
        p('vx_stddev', 0.05)
        p('wz_stddev', 0.02)

        g = self.get_parameter
        self.vesc_topic = str(g('vesc_odom_topic').value)
        self.imu_topic = str(g('imu_topic').value)
        self.odom_topic = str(g('odom_topic').value)
        self.odom_frame = str(g('odom_frame').value)
        self.base_frame = str(g('base_frame').value)
        self.publish_tf = bool(g('publish_tf').value)
        self.publish_rate = float(g('publish_rate').value)
        self.yaw_mode = str(g('yaw_mode').value).lower()
        self.yaw_offset = math.radians(float(g('yaw_offset_deg').value))
        self.invert_yaw = bool(g('invert_yaw').value)
        self.imu_timeout = float(g('imu_timeout').value)
        self.vesc_timeout = float(g('vesc_timeout').value)
        self.min_speed = float(g('min_speed_for_integration').value)
        self.xy_std_per_m = float(g('pose_xy_stddev_per_m').value)
        self.yaw_std = float(g('yaw_stddev').value)
        self.vx_std = float(g('vx_stddev').value)
        self.wz_std = float(g('wz_stddev').value)

        if self.yaw_mode not in ('absolute', 'relative'):
            raise ValueError("yaw_mode должен быть 'absolute' или 'relative'")

        # ---------------------------------------------------------- состояние
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0                 # текущий курс (уже с поправками)
        self.yaw_ref = None            # ноль для режима relative
        self.imu_yaw_raw = None        # последний курс IMU (после offset/inversion)
        self.prev_yaw = None           # курс на предыдущем такте VESC
        self.imu_has_orientation = False
        self.wz = 0.0
        self.vx = 0.0
        self.distance = 0.0
        self.last_imu_time = None      # rclpy Time
        self.last_vesc_time = None
        self.last_imu_stamp_s = None   # для интеграции гироскопа
        self._warned_imu = False
        self._warned_vesc = False

        # ---------------------------------------------------------- топики
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        # /odom/vesc публикуется RELIABLE; подписка BEST_EFFORT совместима.
        vesc_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(Imu, self.imu_topic, self._on_imu, sensor_qos)
        self.create_subscription(Odometry, self.vesc_topic, self._on_vesc, vesc_qos)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_service(Trigger, '~/reset_odom', self._on_reset)

        if self.publish_rate > 0.0:
            self.create_timer(1.0 / self.publish_rate, self._publish)
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f'robot_odom: путь/скорость <- {self.vesc_topic}, курс <- {self.imu_topic} '
            f'(yaw_mode={self.yaw_mode}, offset={math.degrees(self.yaw_offset):.1f}°, '
            f'publish_tf={self.publish_tf}) -> {self.odom_topic}'
        )

    # ------------------------------------------------------------- сервисы
    def _on_reset(self, _req, resp):
        self.x = 0.0
        self.y = 0.0
        self.distance = 0.0
        if self.yaw_mode == 'relative':
            self.yaw_ref = self.imu_yaw_raw
            self.yaw = 0.0
        resp.success = True
        resp.message = 'Одометрия сброшена'
        self.get_logger().info(resp.message)
        return resp

    # ------------------------------------------------------------- IMU
    def _on_imu(self, msg: Imu):
        now = self.get_clock().now()
        self.last_imu_time = now
        self.wz = float(msg.angular_velocity.z)
        if self.invert_yaw:
            self.wz = -self.wz

        q = msg.orientation
        # По REP-145 orientation_covariance[0] == -1 означает «ориентации нет».
        has_orient = (msg.orientation_covariance[0] >= 0.0
                      and (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 0.5)

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if has_orient:
            self.imu_has_orientation = True
            yaw_raw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            if self.invert_yaw:
                yaw_raw = -yaw_raw
            yaw_raw = normalize_angle(yaw_raw + self.yaw_offset)
            self.imu_yaw_raw = yaw_raw

            if self.yaw_mode == 'relative':
                if self.yaw_ref is None:
                    self.yaw_ref = yaw_raw
                    self.get_logger().info(
                        f'Нулевой курс зафиксирован: {math.degrees(yaw_raw):.1f}° (ENU)')
                self.yaw = normalize_angle(yaw_raw - self.yaw_ref)
            else:
                self.yaw = yaw_raw
        else:
            # Ориентации нет — курс НЕ трогаем и гироскоп НЕ интегрируем:
            # угол берётся только из готового кватерниона IMU.
            self.imu_has_orientation = False
            if not self._warned_imu:
                self.get_logger().warning(
                    'IMU не даёт ориентацию — позиция не интегрируется. '
                    'Дождитесь калибровки imu_stm32_bridge.')
                self._warned_imu = True

        self.last_imu_stamp_s = stamp_s

    # ------------------------------------------------------------- VESC
    def _on_vesc(self, msg: Odometry):
        now = self.get_clock().now()
        vx = float(msg.twist.twist.linear.x)
        if abs(vx) < self.min_speed:
            vx = 0.0

        if self.last_vesc_time is not None:
            dt = (now - self.last_vesc_time).nanoseconds * 1e-9
            imu_fresh = (self.last_imu_time is not None
                         and (now - self.last_imu_time).nanoseconds * 1e-9 < self.imu_timeout)
            if 0.0 < dt < 1.0 and imu_fresh and self.imu_has_orientation:
                # Средний курс за такт — по двум последовательным кватернионам.
                mid_yaw = self.yaw
                if self.prev_yaw is not None:
                    mid_yaw = self.yaw + 0.5 * normalize_angle(self.prev_yaw - self.yaw)
                ds = 0.5 * (self.vx + vx) * dt
                self.x += ds * math.cos(mid_yaw)
                self.y += ds * math.sin(mid_yaw)
                self.distance += abs(ds)
            self.prev_yaw = self.yaw

        self.vx = vx
        self.last_vesc_time = now

        if self.publish_rate <= 0.0:
            self._publish()

    # ------------------------------------------------------------- выход
    def _publish(self):
        now = self.get_clock().now()
        imu_fresh = (self.last_imu_time is not None
                     and (now - self.last_imu_time).nanoseconds * 1e-9 < self.imu_timeout)
        vesc_fresh = (self.last_vesc_time is not None
                      and (now - self.last_vesc_time).nanoseconds * 1e-9 < self.vesc_timeout)

        vx = self.vx if vesc_fresh else 0.0
        wz = self.wz if imu_fresh else 0.0

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        xy_var = (0.05 + self.xy_std_per_m * self.distance) ** 2
        yaw_var = self.yaw_std ** 2 if imu_fresh else 1.0e3
        pose_cov = [0.0] * 36
        pose_cov[0] = xy_var
        pose_cov[7] = xy_var
        pose_cov[14] = 1.0e6
        pose_cov[21] = 1.0e6
        pose_cov[28] = 1.0e6
        pose_cov[35] = yaw_var
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = self.vx_std ** 2 if vesc_fresh else 1.0e3
        twist_cov[7] = 1.0e-3   # vy: робот не едет боком
        twist_cov[14] = 1.0e6
        twist_cov[21] = 1.0e6
        twist_cov[28] = 1.0e6
        twist_cov[35] = self.wz_std ** 2 if imu_fresh else 1.0e3
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def _watchdog(self):
        now = self.get_clock().now()
        if self.last_imu_time is None or \
                (now - self.last_imu_time).nanoseconds * 1e-9 > self.imu_timeout:
            self.get_logger().warning(
                f'Нет данных IMU ({self.imu_topic}) — интеграция позиции остановлена',
                throttle_duration_sec=5.0)
        if self.last_vesc_time is None or \
                (now - self.last_vesc_time).nanoseconds * 1e-9 > self.vesc_timeout:
            self.get_logger().warning(
                f'Нет данных VESC ({self.vesc_topic})', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
