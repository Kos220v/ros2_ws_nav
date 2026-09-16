# robot_navigation — автономная навигация гусеничного робота

Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy

Робот умеет:

* **ехать автономно с объездом препятствий** по лидару (Nav2: костмапы +
  планировщик + контроллер + независимый `collision_monitor`);
* **ехать в указанную точку** — по GPS-координатам, в метрах карты или
  «на 5 м вперёд и 2 м влево» (`goto_point`);
* **проезжать маршрут из GPS-точек** (`gps_waypoint_commander` через экшен
  Nav2 `/follow_gps_waypoints`), в том числе по тумблеру пульта.

Одометрия построена так:

| Величина | Источник | Кто НЕ используется |
|---|---|---|
| пройденный путь, линейная скорость | тахометры **VESC** (`kolesa_control` → `/odom/vesc`) | — |
| угол поворота (yaw), угловая скорость | **IMU STM32** (`imu_stm32_bridge` → `/imu/data`) | разность гусениц — нигде |
| привязка к местности | GNSS (`/gps/fix` → `navsat_transform`) | — |

---

## 1. Как это устроено

```
 ┌─────────── СЛОЙ ЖЕЛЕЗА (project_start/start.launch.py) ──────────────────┐
 │ пульт ELRS   ──> /cmd_vel/manual, /control_mode                          │
 │ 2×VESC       ──> kolesa_control ──> /odom/vesc  (только vx, м/с)         │
 │ STM32 IMU    ──> imu_stm32_bridge ──> /imu/data (кватернион ENU, gyro)    │
 │ /odom/vesc + /imu/data ──> robot_odom ──> /odom  (x,y = ∫ vx·(cos,sin)yaw)│
 │ GNSS         ──> /gps/fix                                                 │
 │ лидар        ──> /scan ──> relay ──> /scan_reliable                       │
 │ URDF         ──> TF base_link -> imu_link / laser_frame / gps_link        │
 └───────────────────────────────────────────────────────────────────────────┘
                                   │
 ┌─────────── ЛОКАЛИЗАЦИЯ (localization.launch.py) ─────────────────────────┐
 │ /odom(vx) + /imu/data(yaw,wz)              ─> ekf_odom ─> TF odom→base_link│
 │ /odom(vx) + /imu/data + /odometry/gps      ─> ekf_map  ─> TF map→odom      │
 │ /gps/fix + /imu/data + /odometry/global    ─> navsat_transform ─> /odometry/gps, /fromLL │
 │ (use_gps:=false: ekf_map и navsat не запускаются, map == odom статически) │
 └───────────────────────────────────────────────────────────────────────────┘
                                   │
 ┌─────────── НАВИГАЦИЯ (navigation.launch.py) ─────────────────────────────┐
 │ planner_server (NavFn) + controller_server (Regulated Pure Pursuit)       │
 │ костмапы по /scan_reliable, behavior_server (восстановление)              │
 │ waypoint_follower  ─ экшен /follow_gps_waypoints                          │
 │ velocity_smoother ─> collision_monitor (стоп-зона по лидару)              │
 │        ─> /cmd_vel/auto ─> cmd_switcher ─> /cmd_vel ─> kolesa_control     │
 └───────────────────────────────────────────────────────────────────────────┘
                                   │
   gps_waypoint_commander (маршрут YAML)      goto_point (одна точка)
```

### Почему так

**Курс только с IMU.** Гусеницы в повороте буксуют, поэтому разность тиков
бортов даёт угол с ошибкой в десятки процентов, и она накапливается. Прошивка
STM32 (MPU6050 + QMC5883L) сама сливает гироскоп, акселерометр и компас и
отдаёт абсолютный курс в ENU с учётом магнитного склонения. `kolesa_control`
угол не считает вообще; `robot_odom` интегрирует путь VESC вдоль курса IMU.

**Два EKF.** Локальный (`odom → base_link`) гладкий и непрерывный — по нему
ведёт контроллер. Глобальный (`map → odom`) привязан к GPS и может скакнуть
на метр при новом фиксе; скачок уходит в трансформ `map → odom` и не рвёт
траекторию под гусеницами.

**Nav2 не подключён к `/cmd_vel` напрямую.** Выход идёт в `/cmd_vel/auto`,
`cmd_switcher` отдаёт приоритет пульту: оператор перехватывает управление,
просто двинув стик.

**Три уровня объезда препятствий.** Глобальный планировщик обходит то, что
видно в костмапе; контроллер RPP проверяет столкновение по траектории и
сбрасывает скорость у препятствий; `collision_monitor` независимо
останавливает робота, если в стоп-зоне перед гусеницами что-то появилось.

---

## 2. Установка зависимостей

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-robot-localization \
  ros-jazzy-nmea-navsat-driver \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-collision-monitor \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rmw-cyclonedds-cpp \
  python3-serial python3-yaml
```

Лидар: SDK YDLidar (`ydlidar_sdk`) должен быть установлен отдельно, как
описано в `src/ydlidar_ros2_driver/README.md`.

DDS (один раз): полный стек — ~25 процессов, CycloneDDS по умолчанию столько
участников без multicast не выдаёт.

```bash
cd ~/ros2_ws_nav
./setup_dds.sh && source ~/.bashrc
```

Сборка:

```bash
cd ~/ros2_ws_nav
./rebuild.sh            # чистая сборка одним режимом
source install/setup.bash
```

udev-правила для стабильных имён портов: `/dev/imu_stm32`
(`src/imu_stm32_bridge/udev/`), `/dev/lidar`, `/dev/gps` — см. п. 4.

---

## 3. Ввод в эксплуатацию

Каждый шаг проверяется отдельно.

### Шаг 1. Калибровка IMU (в прошивке STM32)

```bash
ros2 launch imu_stm32_bridge imu.launch.py port:=/dev/imu_stm32
ros2 service call /imu/imu_stm32_bridge/gyro_calib std_srvs/srv/Trigger       # не трогать 2 с
ros2 service call /imu/imu_stm32_bridge/mag_calib_start std_srvs/srv/Trigger  # вращать плату ~30 с
ros2 service call /imu/imu_stm32_bridge/mag_calib_stop_save std_srvs/srv/Trigger
ros2 service call /imu/imu_stm32_bridge/save_flash std_srvs/srv/Trigger
```

Калибруйте магнитометр **на роботе, вдали от моторов включённых** — так
компенсируется искажение поля корпусом. Подробнее:
`src/imu_stm32_bridge/docs/CALIBRATION.md`.

Магнитное склонение своей местности задайте аргументом `declination_deg`
(https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml). Оно уходит в
прошивку; в `navsat_transform` стоит 0 — не дублируйте.

### Шаг 2. Угол монтажа платы

```bash
ros2 launch project_start start.launch.py use_gps:=false odom_publish_tf:=true
ros2 run robot_navigation heading_check
```

Поставьте робота носом на север: ожидаемый `IMU yaw = 90° ENU`. Разницу
впишите в `imu_yaw_offset_deg` (аргумент launch) **и** в `imu_rpy` в
`tracked_robot_description/urdf/tracked_robot.urdf.xacro` (EKF поворачивает
данные IMU по TF `base_link → imu_link`, а `robot_odom` — по параметру; оба
должны совпадать).

### Шаг 3. Одометрия VESC

Калибровка `tacho_counts_per_revolution` / `distance_per_revolution` —
`src/kolesa_control/README.md`. Проверка: проехать пультом 5 м по прямой,
`ros2 topic echo /odom` → `pose.position.x ≈ 5`. Повернуть на месте на 90° —
`x`, `y` почти не меняются, yaw меняется на 1.57 (это IMU).

### Шаг 4. Локализация без движения

```bash
ros2 launch robot_navigation bringup.launch.py use_navigation:=false
ros2 topic hz /odom /imu/data /gps/fix /odometry/local /odometry/global
ros2 topic echo /gps/filtered      # должна совпадать с /gps/fix в пределах метров
ros2 run tf2_tools view_frames     # map -> odom -> base_link -> датчики
```

### Шаг 5. Проверка перед выездом и пробная поездка

```bash
ros2 launch robot_navigation bringup.launch.py declination_deg:=11.9
ros2 run robot_navigation nav_preflight_check          # всё OK?
ros2 run robot_navigation goto_point -- --dx 3.0       # 3 м вперёд с объездом
```

Робот поедет только когда `/cmd_vel/manual` молчит (стики в нейтрали, или
тумблер не в MANUAL). Держите палец на стике: любое движение — перехват.

### Шаг 6. Запись маршрута

```bash
ros2 run robot_navigation gps_waypoint_logger --ros-args -p output_file:=/home/pi/route.yaml
# катаем пультом, в нужных местах:
ros2 service call /gps_waypoint_logger/log_waypoint std_srvs/srv/Trigger
# или автоматически каждые 10 м:  -p auto_interval_m:=10.0
```

Точки каждые 5–20 м и на каждом повороте; между соседними точками < 55 м
(половина скользящего глобального костмапа).

### Шаг 7. Маршрут

```bash
ros2 launch robot_navigation bringup.launch.py waypoints_file:=/home/pi/route.yaml
# старт: тумблер пульта в AUTO  или
ros2 service call /gps_waypoint_commander/start_route std_srvs/srv/Trigger
# состояние
ros2 topic echo /gps_waypoint_commander/status
# стоп / домой
ros2 service call /gps_waypoint_commander/stop_route std_srvs/srv/Trigger
ros2 service call /gps_waypoint_commander/return_home std_srvs/srv/Trigger
```

---

## 4. Поездка в точку (`goto_point`)

```bash
# GPS
ros2 run robot_navigation goto_point -- --lat 56.299145 --lon 43.923061
# в метрах фрейма map
ros2 run robot_navigation goto_point -- --x 12.0 --y -3.5 --yaw 1.57
# относительно робота
ros2 run robot_navigation goto_point -- --dx 5 --dy -2
```

В составе `bringup` узел `goto_point` уже запущен и принимает цели по топикам
(удобно из приложения через rosbridge):

```bash
ros2 topic pub --once /goto/gps geographic_msgs/msg/GeoPoint "{latitude: 56.299145, longitude: 43.923061}"
ros2 topic pub --once /goto/pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 4.0}, orientation: {w: 1.0}}}"
ros2 topic pub --once /goto/cancel std_msgs/msg/Empty
```

Без GPS (`use_gps:=false`) работают режимы `--x/--y` и `--dx/--dy`:
`map` совпадает с `odom`, начало — место включения.

---

## 5. Настройка под свой робот

| Что | Где |
|---|---|
| порты VESC, инверсии, `duty_max`, калибровка тахометра | `project_start/launch/start.launch.py` (`kolesa_control_node`) |
| порт/скорость GNSS | аргументы `gps_port`, `gps_baud` |
| склонение, угол монтажа IMU | аргументы `declination_deg`, `imu_yaw_offset_deg` |
| положение датчиков (TF) | `tracked_robot_description/urdf/tracked_robot.urdf.xacro` |
| габариты (`robot_radius`, стоп-зоны) | `config/nav2_params.yaml` (`robot_radius`, `collision_monitor`) |
| скорости | `nav2_params.yaml`: `desired_linear_vel`, `velocity_smoother.max_velocity`; `kolesa_control.max_linear_velocity` |
| доверие к датчикам | `config/dual_ekf_navsat.yaml`, `robot_odom/config/odom_params.yaml` |

udev для лидара/GPS (пример, подставьте свои `idVendor`/`idProduct` из
`udevadm info -a -n /dev/ttyUSB0`):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps"
```

---

## 6. Диагностика

**Робот не едет по команде Nav2.** `ros2 topic echo /cmd_vel/auto` — есть
команды? Если нет — `ros2 topic echo /collision_monitor_state` (стоп-зона?),
`ros2 lifecycle get /controller_server` (active?). Если есть — `ros2 topic
echo /cmd_vel/manual`: пульт держит канал (стики не в нейтрали).

**Курс уплывает / робот едет «змейкой».** `heading_check`: сравните IMU yaw
с реальным направлением в 4 сторонах света. Ошибка одинаковая — поправьте
`imu_yaw_offset_deg`; разная — перекалибруйте магнитометр на роботе,
проверьте, что рядом с платой нет силовых проводов/моторов. Режим `6x` в
логе `imu_stm32_bridge` означает, что магнитометр не работает.

**`/odom` стоит на месте, хотя робот едет.** `ros2 topic echo /odom/vesc` —
если `twist.linear.x = 0`, проблема в VESC-телеметрии (`/kolesa/diagnostics`:
`connected`, `telemetry_age_s`). Если скорость есть, а `/odom` нет — нет
`/imu/data` (robot_odom замораживает интеграцию без курса).

**GOAL_OUTSIDE_MAP.** Точка дальше ~60 м от робота — добавьте промежуточные.

**`Failed to find a free participant index`.** Не выполнен `setup_dds.sh`
или живы процессы прошлых запусков: `pkill -f 'ros2 launch'`.

**Pi4 не тянет.** `controller_frequency: 10`, `local_costmap.update_frequency:
3`, разрешение глобального костмапа `0.2`.

Полезно:

```bash
ros2 run robot_navigation nav_preflight_check
ros2 topic echo /kolesa/diagnostics
ros2 run tf2_tools view_frames
ros2 lifecycle get /waypoint_follower
```
