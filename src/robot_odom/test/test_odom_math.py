import math

from robot_odom.odom_node import normalize_angle, quaternion_from_yaw, yaw_from_quaternion


def test_yaw_roundtrip():
    for deg in (-179.0, -90.0, 0.0, 45.0, 90.0, 179.0):
        yaw = math.radians(deg)
        q = quaternion_from_yaw(yaw)
        assert abs(normalize_angle(yaw_from_quaternion(q.x, q.y, q.z, q.w) - yaw)) < 1e-9


def test_normalize():
    assert abs(normalize_angle(3 * math.pi) - math.pi) < 1e-9 or \
        abs(normalize_angle(3 * math.pi) + math.pi) < 1e-9
    assert abs(normalize_angle(0.5) - 0.5) < 1e-12
