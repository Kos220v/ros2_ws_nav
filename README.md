# ros2_ws_nav

ROS 2 Jazzy · Raspberry Pi 4 · Ubuntu 24.04 — гусеничный робот с автономной
навигацией.

* одометрия: путь/скорость — **VESC** (`kolesa_control`), курс — **IMU STM32**
  (`imu_stm32_bridge`), объединение — `robot_odom`;
* локализация: `robot_localization` (два EKF + navsat_transform);
* навигация: Nav2 — автономный объезд препятствий по лидару, поездка в точку
  (`goto_point`), маршрут по GPS (`gps_waypoint_commander`).

Быстрый старт и полная документация: [`src/robot_navigation/README.md`](src/robot_navigation/README.md).

```bash
./setup_dds.sh && source ~/.bashrc
./rebuild.sh && source install/setup.bash
ros2 launch robot_navigation bringup.launch.py declination_deg:=11.9 waypoints_file:=/home/pi/route.yaml
```
