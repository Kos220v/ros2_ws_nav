# -*- coding: utf-8 -*-

"""
Нода kolesa_control

Дифференциальное управление гусеничным роботом через два контроллера
FS75100 / VESC по UART.

ОДОМЕТРИЯ
---------
Узел считает ТОЛЬКО то, что достоверно измеряет VESC: пройденный путь и
линейную скорость каждой гусеницы по абсолютному тахометру. Угол поворота
(yaw) здесь НЕ вычисляется — гусеничная машина в повороте проскальзывает,
и разность тиков бортов не имеет отношения к реальному курсу. Курс берётся
с инерциального модуля (imu_stm32_bridge) узлом robot_odom, который
объединяет дистанцию VESC и ориентацию IMU в /odom.

Подписки:
  /cmd_vel            geometry_msgs/Twist

Публикации:
  /odom/vesc          nav_msgs/Odometry
      Только скорость: twist.linear.x — скорость центра робота, м/с.
      Поза и угловая скорость намеренно не заполняются (ковариация 1e6),
      чтобы их никто случайно не «сфьюзил».
  /joint_states       sensor_msgs/JointState
      Положение (рад) и скорость (рад/с) левой и правой гусениц.
  /kolesa/diagnostics diagnostic_msgs/DiagnosticArray
      Напряжение, скважность, обороты, тики, пройденный путь по бортам.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .vesc_driver import VescDriver


# Ковариация «это значение не измерено, не используйте его».
UNMEASURED_COV = 1.0e6


def velocity_to_duty_cycle(velocity, max_velocity, duty_min=0.03, duty_max=1.0):
    """
    Преобразует целевую скорость в скважность (duty cycle) для VESC.

    Направление уже учтено в знаке velocity; функция ничего не знает
    про invert_left/right.
    """
    if abs(velocity) < 0.001:
        return 0.0
    normalized = abs(velocity) / max_velocity
    normalized = min(1.0, max(0.0, normalized))
    duty_magnitude = duty_min + normalized * (duty_max - duty_min)
    return duty_magnitude if velocity >= 0 else -duty_magnitude


def delta_i32(current, previous):
    """Разница двух int32-счётчиков с учётом переполнения."""
    current = int(current)
    previous = int(previous)
    delta = current - previous
    if delta > 2147483647:
        delta -= 4294967296
    elif delta < -2147483648:
        delta += 4294967296
    return delta


class KolesaControl(Node):
    def __init__(self):
        super().__init__("kolesa_control")

        # ----------------------------------------------------------- параметры
        p = self.declare_parameter

        p("left_port", "/dev/ttyAMA4")
        p("right_port", "/dev/ttyAMA5")
        p("baud", 115200)

        # Геометрия шасси (нужна только для раскладки cmd_vel по бортам)
        p("wheel_separation", 0.48)

        # Калибровка одометрии по оборотам выходного вала
        p("tacho_counts_per_revolution", 2157.0)
        p("distance_per_revolution", 2.011)
        p("odometry_scale", 1.0)

        # Инверсии
        p("invert_left", False)
        p("invert_right", False)
        p("encoder_invert_left", False)
        p("encoder_invert_right", True)
        p("invert_angular", False)

        # Скважность (duty cycle) VESC
        p("duty_min", 0.03)
        p("duty_max", 1.0)
        p("max_linear_velocity", 1.0)
        p("max_angular_velocity", 1.0)
        p("control_rate", 50.0)
        p("telemetry_rate", 20.0)
        p("cmd_timeout", 0.5)
        p("telemetry_stale_timeout", 0.5)

        # Фильтрация скачков тахометра
        p("tacho_jump_margin", 3.0)
        p("min_tacho_jump_threshold", 500.0)

        # Публикации
        p("publish_odom", True)
        p("odom_topic", "odom/vesc")
        p("odom_frame", "odom")
        p("base_frame", "base_link")
        p("publish_joint_states", True)
        p("left_wheel_joint", "left_track_joint")
        p("right_wheel_joint", "right_track_joint")
        p("publish_diagnostics", True)

        g = self.get_parameter

        self.left_port = str(g("left_port").value)
        self.right_port = str(g("right_port").value)
        self.baud = int(g("baud").value)

        self.separation = float(g("wheel_separation").value)
        self.tacho_counts_per_revolution = float(g("tacho_counts_per_revolution").value)
        self.distance_per_revolution = float(g("distance_per_revolution").value)
        self.odometry_scale = float(g("odometry_scale").value)

        self.radius = self.distance_per_revolution / (2.0 * math.pi)

        self.kin_inv_left = -1 if g("invert_left").value else 1
        self.kin_inv_right = -1 if g("invert_right").value else 1
        self.enc_inv_left = -1 if g("encoder_invert_left").value else 1
        self.enc_inv_right = -1 if g("encoder_invert_right").value else 1
        self.invert_angular = bool(g("invert_angular").value)

        self.duty_min = float(g("duty_min").value)
        self.duty_max = float(g("duty_max").value)
        self.max_linear_velocity = float(g("max_linear_velocity").value)
        self.max_angular_velocity = float(g("max_angular_velocity").value)
        self.control_rate = float(g("control_rate").value)
        self.telemetry_rate = float(g("telemetry_rate").value)
        self.cmd_timeout = float(g("cmd_timeout").value)
        self.telemetry_stale_timeout = float(g("telemetry_stale_timeout").value)
        self.tacho_jump_margin = float(g("tacho_jump_margin").value)
        self.min_tacho_jump_threshold = float(g("min_tacho_jump_threshold").value)

        self.pub_odom = bool(g("publish_odom").value)
        self.odom_topic = str(g("odom_topic").value)
        self.odom_frame = str(g("odom_frame").value)
        self.base_frame = str(g("base_frame").value)
        self.pub_js = bool(g("publish_joint_states").value)
        self.left_joint = str(g("left_wheel_joint").value)
        self.right_joint = str(g("right_wheel_joint").value)
        self.pub_diag = bool(g("publish_diagnostics").value)

        self._validate_params()

        self.distance_per_tacho_count = (
            self.distance_per_revolution / self.tacho_counts_per_revolution
        )
        self.rad_per_tacho_count = (2.0 * math.pi) / self.tacho_counts_per_revolution

        self.get_logger().info("=" * 60)
        self.get_logger().info("КОНФИГУРАЦИЯ ПРИВОДА (DUTY CYCLE)")
        self.get_logger().info(f"  Радиус: {self.radius:.4f} м | База: {self.separation:.4f} м")
        self.get_logger().info(f"  Инверсия L/R: {self.kin_inv_left}/{self.kin_inv_right} | "
                               f"invert_angular: {self.invert_angular}")
        self.get_logger().info(f"  Макс. линейная: {self.max_linear_velocity:.2f} м/с")
        self.get_logger().info(f"  Скважность: min={self.duty_min:.3f} max={self.duty_max:.3f}")
        self.get_logger().info(
            f"  Одометрия: {self.tacho_counts_per_revolution:.2f} тиков/оборот, "
            f"{self.distance_per_revolution:.4f} м/оборот "
            f"({self.distance_per_tacho_count * 1000.0:.5f} мм/тик)"
        )
        self.get_logger().info(
            "  Курс (yaw) по гусеницам НЕ считается — его даёт IMU (robot_odom)."
        )
        self.get_logger().info("=" * 60)

        # ----------------------------------------------------------- драйверы
        self.left = VescDriver("left", self.left_port, self.baud, self.get_logger())
        self.right = VescDriver("right", self.right_port, self.baud, self.get_logger())
        self.left.start()
        self.right.start()

        # ----------------------------------------------------------- состояние
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.wheels = {
            "left": self._make_wheel_state(),
            "right": self._make_wheel_state(),
        }

        # ----------------------------------------------------------- топики
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        if self.pub_odom:
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        if self.pub_js:
            self.js_pub = self.create_publisher(JointState, "joint_states", 10)
        if self.pub_diag:
            self.diag_pub = self.create_publisher(
                DiagnosticArray, "kolesa/diagnostics", qos_profile_sensor_data,
            )

        # ----------------------------------------------------------- таймеры
        self.create_timer(1.0 / self.control_rate, self._control_tick)
        self.create_timer(1.0 / self.telemetry_rate, self._telemetry_tick)

        self.get_logger().info("kolesa_control запущена (абсолютный тахометр VESC)")

    # ------------------------------------------------------------- параметры
    def _validate_params(self):
        if (
            self.radius <= 0.0
            or self.separation <= 0.0
            or self.control_rate <= 0.0
            or self.telemetry_rate <= 0.0
            or self.max_linear_velocity <= 0.0
            or self.max_angular_velocity <= 0.0
        ):
            raise ValueError("Некорректные параметры конфигурации геометрии или скорости")
        if self.tacho_counts_per_revolution <= 0.0:
            raise ValueError("tacho_counts_per_revolution должен быть > 0")
        if self.distance_per_revolution <= 0.0:
            raise ValueError("distance_per_revolution должен быть > 0")
        if self.duty_min < 0.0 or self.duty_max <= 0.0 or self.duty_min >= self.duty_max:
            raise ValueError("Некорректные duty_min/duty_max (должно быть 0 <= duty_min < duty_max)")
        if self.duty_max > 1.0:
            raise ValueError("duty_max не может быть больше 1.0 (100% скважности)")

    def _make_wheel_state(self):
        return {
            "pos": 0.0, "distance": 0.0, "speed": 0.0, "omega": 0.0,
            "raw_tacho": None, "raw_tacho_abs": None,
            "prev_tacho": None,
            "initial_tacho": None,
            "total_abs_counts": 0,
            "last_delta_counts": 0, "erpm": 0.0,
            "duty_measured": 0.0, "duty_target": 0.0,
            "voltage": 0.0, "current_motor": 0.0, "temp_fet": 0.0,
            "fault": 0,
            "last_rx_time": None, "last_tacho_time": None,
            "telemetry_age": float("inf"), "stale": True,
        }

    # ------------------------------------------------------------- callbacks
    def _on_cmd_vel(self, msg: Twist):
        self.cmd_v = float(msg.linear.x)
        self.cmd_w = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def _control_tick(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9

        if dt > self.cmd_timeout:
            v = 0.0
            w = 0.0
        else:
            v = self.cmd_v
            w = self.cmd_w

        v = max(-self.max_linear_velocity, min(self.max_linear_velocity, v))
        w = max(-self.max_angular_velocity, min(self.max_angular_velocity, w))

        if self.invert_angular:
            w = -w

        half_sep = self.separation / 2.0

        v_left_target = v - (w * half_sep)
        v_right_target = v + (w * half_sep)

        v_left_final = v_left_target * self.kin_inv_left
        v_right_final = v_right_target * self.kin_inv_right

        duty_left = velocity_to_duty_cycle(
            v_left_final, self.max_linear_velocity, self.duty_min, self.duty_max,
        )
        duty_right = velocity_to_duty_cycle(
            v_right_final, self.max_linear_velocity, self.duty_min, self.duty_max,
        )

        self.wheels["left"]["duty_target"] = duty_left
        self.wheels["right"]["duty_target"] = duty_right

        self.left.set_duty(duty_left)
        self.right.set_duty(duty_right)

    def _telemetry_tick(self):
        self.left.request_telemetry()
        self.right.request_telemetry()

        tl = self.left.get_telemetry()
        tr = self.right.get_telemetry()

        self._update_wheel_from_tacho("left", tl, direction_sign=self.enc_inv_left)
        self._update_wheel_from_tacho("right", tr, direction_sign=self.enc_inv_right)
        self._update_stale_state("left")
        self._update_stale_state("right")

        if self.pub_odom:
            self._publish_odom()
        if self.pub_js:
            self._publish_joint_states()
        if self.pub_diag:
            self._publish_diagnostics()

    def _update_wheel_from_tacho(self, side, telemetry, direction_sign):
        st = self.wheels[side]

        if telemetry is None:
            return

        if "tachometer" not in telemetry:
            return

        rx_time_raw = telemetry.get("_rx_time", None)
        if rx_time_raw is not None:
            rx_time = float(rx_time_raw)
            if st["last_rx_time"] == rx_time:
                return
        else:
            rx_time = time.monotonic()

        st["last_rx_time"] = rx_time
        st["telemetry_age"] = 0.0
        st["stale"] = False

        current_tacho = int(telemetry["tachometer"])
        st["raw_tacho"] = current_tacho

        if "tachometer_abs" in telemetry:
            st["raw_tacho_abs"] = int(telemetry["tachometer_abs"])
        elif "tacho_abs" in telemetry:
            st["raw_tacho_abs"] = int(telemetry["tacho_abs"])

        raw_erpm = telemetry.get("erpm", telemetry.get("rpm", 0.0))
        st["erpm"] = float(raw_erpm)
        st["duty_measured"] = float(telemetry.get("duty", 0.0))
        st["voltage"] = float(telemetry.get("v_in", telemetry.get("voltage", 0.0)) or 0.0)
        st["current_motor"] = float(telemetry.get("current_motor", 0.0) or 0.0)
        st["temp_fet"] = float(telemetry.get("temp_fet", telemetry.get("temp_mos", 0.0)) or 0.0)
        st["fault"] = int(telemetry.get("fault", telemetry.get("fault_code", 0)) or 0)

        if st["initial_tacho"] is None:
            st["initial_tacho"] = current_tacho
            st["prev_tacho"] = current_tacho
            st["last_tacho_time"] = rx_time
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["last_delta_counts"] = 0
            self.get_logger().info(f"[{side}] initial_tacho = {current_tacho}")
            return

        offset = delta_i32(current_tacho, st["initial_tacho"]) * direction_sign
        st["pos"] = offset * self.rad_per_tacho_count * self.odometry_scale
        st["distance"] = offset * self.distance_per_tacho_count * self.odometry_scale

        dt = rx_time - st["last_tacho_time"]
        delta_raw = delta_i32(current_tacho, st["prev_tacho"])
        st["prev_tacho"] = current_tacho
        st["last_tacho_time"] = rx_time

        if not self._tacho_delta_is_reasonable(delta_raw, dt):
            self.get_logger().warning(
                f"[{side}] подозрительный скачок tachometer: "
                f"delta={delta_raw}, dt={dt:.3f} c."
            )
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["last_delta_counts"] = 0
            return

        delta = delta_raw * direction_sign
        st["last_delta_counts"] = delta
        st["total_abs_counts"] += abs(delta_raw)

        if dt > 1e-6:
            distance_delta = delta * self.distance_per_tacho_count * self.odometry_scale
            angle_delta = delta * self.rad_per_tacho_count * self.odometry_scale
            st["omega"] = angle_delta / dt
            st["speed"] = distance_delta / dt
        else:
            st["omega"] = 0.0
            st["speed"] = 0.0

    def _tacho_delta_is_reasonable(self, delta_counts, dt):
        if dt <= 0.0:
            return True
        min_dt = 1.0 / self.telemetry_rate
        max_counts_per_sec = self.max_linear_velocity / self.distance_per_tacho_count
        threshold = max(
            self.min_tacho_jump_threshold,
            max_counts_per_sec * max(dt, min_dt) * self.tacho_jump_margin,
        )
        return abs(delta_counts) <= threshold

    def _update_stale_state(self, side):
        st = self.wheels[side]
        if st["last_rx_time"] is None:
            st["telemetry_age"] = float("inf")
            st["stale"] = True
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["erpm"] = 0.0
            st["duty_measured"] = 0.0
            return

        age = time.monotonic() - st["last_rx_time"]
        st["telemetry_age"] = age
        if age > self.telemetry_stale_timeout:
            st["stale"] = True
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["erpm"] = 0.0
            st["duty_measured"] = 0.0
        else:
            st["stale"] = False

    # ------------------------------------------------------------- публикации
    def _both_tracks_valid(self):
        left = self.wheels["left"]
        right = self.wheels["right"]
        return (
            left["initial_tacho"] is not None
            and right["initial_tacho"] is not None
            and not left["stale"]
            and not right["stale"]
        )

    def _publish_odom(self):
        """
        /odom/vesc: ТОЛЬКО линейная скорость центра робота.

        Поза и угловая скорость не измеряются (ковариация 1e6). Курс даёт
        IMU, интеграцию в X/Y выполняет robot_odom.
        """
        left = self.wheels["left"]
        right = self.wheels["right"]

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        valid = self._both_tracks_valid()
        v_center = 0.5 * (left["speed"] + right["speed"]) if valid else 0.0

        msg.twist.twist.linear.x = v_center

        # Все компоненты — «не измерено», кроме vx.
        twist_cov = [0.0] * 36
        for i in range(6):
            twist_cov[i * 6 + i] = UNMEASURED_COV
        twist_cov[0] = 0.01 if valid else UNMEASURED_COV  # vx: ±0.1 м/с
        msg.twist.covariance = twist_cov

        pose_cov = [0.0] * 36
        for i in range(6):
            pose_cov[i * 6 + i] = UNMEASURED_COV
        msg.pose.covariance = pose_cov
        msg.pose.pose.orientation.w = 1.0

        self.odom_pub.publish(msg)

    def _publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.wheels["left"]["pos"], self.wheels["right"]["pos"]]
        js.velocity = [self.wheels["left"]["omega"], self.wheels["right"]["omega"]]
        self.js_pub.publish(js)

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        self._append_wheel_diag(arr, "left", "Левая гусеница", self.left)
        self._append_wheel_diag(arr, "right", "Правая гусеница", self.right)
        self._append_odom_diag(arr)
        self.diag_pub.publish(arr)

    def _append_wheel_diag(self, arr, side, display_name, driver):
        st = self.wheels[side]
        status = DiagnosticStatus()
        status.name = display_name
        status.hardware_id = driver.port

        if not driver.connected:
            status.level = DiagnosticStatus.WARN
            status.message = "Нет соединения с VESC"
        elif st["stale"]:
            status.level = DiagnosticStatus.WARN
            status.message = "Телеметрия устарела"
        elif st["fault"]:
            status.level = DiagnosticStatus.ERROR
            status.message = f"VESC fault code {st['fault']}"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Норма"

        total_abs_revolutions = (
            st["total_abs_counts"] / self.tacho_counts_per_revolution
        )

        kv = status.values.append
        kv(KeyValue(key="connected", value=str(driver.connected)))
        kv(KeyValue(
            key="telemetry_age_s",
            value="inf" if math.isinf(st["telemetry_age"]) else f"{st['telemetry_age']:.3f}",
        ))
        kv(KeyValue(key="raw_tachometer", value=str(st["raw_tacho"])))
        kv(KeyValue(key="raw_tachometer_abs", value=str(st["raw_tacho_abs"])))
        kv(KeyValue(key="initial_tacho", value=str(st["initial_tacho"])))
        kv(KeyValue(key="last_delta_counts", value=str(st["last_delta_counts"])))
        kv(KeyValue(key="total_abs_counts", value=str(st["total_abs_counts"])))
        kv(KeyValue(key="total_abs_revolutions", value=f"{total_abs_revolutions:.2f}"))
        kv(KeyValue(key="distance_m", value=f"{st['distance']:.3f}"))
        kv(KeyValue(key="duty_target", value=f"{st['duty_target']:.3f}"))
        kv(KeyValue(key="duty_measured", value=f"{st['duty_measured']:.3f}"))
        kv(KeyValue(key="erpm", value=f"{st['erpm']:.1f}"))
        kv(KeyValue(key="speed_m_s", value=f"{st['speed']:.3f}"))
        kv(KeyValue(key="omega_rad_s", value=f"{st['omega']:.3f}"))
        kv(KeyValue(key="voltage_v", value=f"{st['voltage']:.2f}"))
        kv(KeyValue(key="current_motor_a", value=f"{st['current_motor']:.2f}"))
        kv(KeyValue(key="temp_fet_c", value=f"{st['temp_fet']:.1f}"))
        kv(KeyValue(key="fault", value=str(st["fault"])))
        arr.status.append(status)

    def _append_odom_diag(self, arr):
        left = self.wheels["left"]
        right = self.wheels["right"]

        status = DiagnosticStatus()
        status.name = "Одометрия VESC (скорость и путь)"
        status.hardware_id = "kolesa_control/odom"

        if not self._both_tracks_valid():
            status.level = DiagnosticStatus.WARN
            status.message = "Нет данных (ожидание телеметрии обоих бортов)"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Норма"

        v_center = 0.5 * (left["speed"] + right["speed"])
        d_center = 0.5 * (left["distance"] + right["distance"])
        # Оценка скорости поворота по разности бортов — ТОЛЬКО для контроля
        # пробуксовки (сравнить с гироскопом). В одометрию не идёт.
        w_tracks_est = (right["speed"] - left["speed"]) / self.separation

        status.values.append(KeyValue(key="v_center_m_s", value=f"{v_center:.3f}"))
        status.values.append(KeyValue(key="distance_center_m", value=f"{d_center:.3f}"))
        status.values.append(KeyValue(
            key="track_yaw_rate_est_rad_s_(diag_only)", value=f"{w_tracks_est:.3f}",
        ))
        arr.status.append(status)

    # ------------------------------------------------------------- завершение
    def shutdown(self):
        try:
            self.left.set_duty(0.0)
            self.right.set_duty(0.0)
        except Exception:  # noqa: BLE001
            pass
        for drv in (self.left, self.right):
            try:
                drv.stop()
            except Exception:  # noqa: BLE001
                pass


def main(args=None):
    rclpy.init(args=args)
    node = KolesaControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Остановка узла по сигналу пользователя")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
