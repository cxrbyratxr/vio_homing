# VIO Homing — текущий прогресс

Обновлено: 02.10.2026

Документ фиксирует текущее состояние стенда и выполненные работы по интеграции Jetson Nano, Pixhawk 6C Mini, PX4, ROS Melodic и VINS-Fusion.

## 1. Аппаратная схема

```text
MacBook
├── Ethernet ─────────────── Jetson Nano
│                            eth0
│                            192.168.137.2/24
│
└── USB-C ───────────────── Pixhawk 6C Mini
                             │
                             └── TELEM2 / UART
                                  │
                                  └── Jetson J41
                                      /dev/ttyTHS1
```

Подключение TELEM2 к Jetson:

```text
Pixhawk TELEM2          Jetson Nano J41
TX pin 2   ---------->  pin 10 RX
RX pin 3   <----------  pin 8  TX
GND pin 6  -----------  pin 6  GND
```

Питание +5 V с TELEM2 на Jetson не используется.

---

## 2. Сеть Mac ↔ Jetson

Статическая сеть:

```text
Jetson: 192.168.137.2/24
Mac:    192.168.137.1/24
Gateway Jetson: 192.168.137.1
```

SSH:

```bash
ssh jetson@192.168.137.2
```

Ethernet используется для разработки, SSH и тестового heartbeat.

Интернет на Jetson не считается гарантированным. При отсутствии доступа изменения Git переносятся на Mac через `git bundle` + `scp`, после чего пушатся в GitHub уже с Mac.

---

## 3. ROS / VINS-Fusion

Jetson:

```text
Ubuntu 18.04.6 LTS
JetPack 4.6.2
ROS Melodic
Python 2.7.17 для ROS
Python 3.6.9 дополнительно
```

Catkin workspace:

```text
/home/jetson/catkin_ws
```

VINS-Fusion:

```text
/home/jetson/catkin_ws/src/VINS-Fusion
```

VINS-Fusion собран. Доступны:

```text
vins_node
loop_fusion_node
global_fusion_node
```

Камера пока физически не подключена, поэтому реальный VINS ещё не запущен в рабочем контуре.

---

## 4. ROS-пакет `vio_homing`

Репозиторий:

```text
/home/jetson/catkin_ws/src/vio_homing
```

Основные файлы:

```text
package.xml
CMakeLists.txt
launch/companion.launch

scripts/
├── mavlink_imu_bridge.py
├── check_imu_timing.py
├── test_odometry.py
├── wifi_link_monitor.py
└── trajectory_recorder.py
```

Сборка:

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

---

## 5. MAVLink Pixhawk ↔ Jetson

UART стабильно работает на:

```text
115200 baud
```

Порты:

```text
Jetson: /dev/ttyTHS1
PX4:    /dev/ttyS3
```

На `921600` наблюдалось большое количество `BAD_DATA`, поэтому рабочая скорость оставлена `115200`.

Подтверждена двусторонняя MAVLink-связь:

```text
Pixhawk -> Jetson
Jetson  -> Pixhawk
```

---

## 6. Постоянная конфигурация PX4

TELEM2 настроен как MAVLink-канал companion computer.

```text
MAV_1_CONFIG     = TELEM 2
MAV_1_MODE       = Custom
MAV_1_RATE       = 10000
MAV_1_FLOW_CTRL  = 0
SER_TEL2_BAUD    = 115200
MAV_PROTO_VER    = 2
```

На microSD Pixhawk создан:

```text
/fs/microsd/etc/extras.txt
```

Текущий набор streams:

```sh
set +e

mavlink stream -d /dev/ttyS3 -s HEARTBEAT -r 1
mavlink stream -d /dev/ttyS3 -s HIGHRES_IMU -r 100
mavlink stream -d /dev/ttyS3 -s SYS_STATUS -r 1
mavlink stream -d /dev/ttyS3 -s LOCAL_POSITION_NED -r 10

set -e
```

После загрузки PX4 `HIGHRES_IMU` поднимается автоматически.

Фактическая частота:

```text
~100 Hz
```

Текущий набор streams — минимальный стендовый. Перед полётными испытаниями нужно определить окончательный набор GPS/attitude/estimator/battery данных с учётом ограничения `115200 baud`.

---

## 7. MAVLink → ROS IMU bridge

Рабочий bridge:

```text
scripts/mavlink_imu_bridge.py
```

Он:

1. открывает `/dev/ttyTHS1` на `115200`;
2. принимает `HIGHRES_IMU`;
3. преобразует MAVLink FRD → ROS FLU;
4. публикует `/pixhawk/imu`;
5. выполняет MAVLink `TIMESYNC`;
6. принимает ROS `Odometry`;
7. отправляет каждое новое odometry-сообщение в PX4 через MAVLink `ODOMETRY`.

Преобразование осей IMU:

```text
MAVLink FRD        ROS FLU
X forward    ->    X forward
Y right      ->   -Y left
Z down       ->   -Z up
```

---

## 8. Проверка IMU timing

Тест 1000 сообщений:

```text
Samples:        1000
Mean dt:        0.010001 s
Mean rate:      99.99 Hz
Std deviation:  0.002325 s
Min dt:         0.004427 s
Max dt:         0.015055 s
Gaps > 15 ms:   1
Bad timestamps: 0
```

Итог:

```text
IMU ~100 Hz
timestamps монотонные
Bad timestamps = 0
```

---

## 9. TIMESYNC

В bridge реализован MAVLink `TIMESYNC`.

Используется медиана последних 30 валидных измерений offset.

Большое абсолютное значение offset ожидаемо, потому что PX4 и Jetson используют разные временные базы.

В тестах offset менялся в пределах нескольких миллисекунд, что достаточно для текущего стендового VIO-прототипа.

---

## 10. ROS Odometry → MAVLink ODOMETRY → PX4

Реализован и проверен канал:

```text
ROS /vio/odom_frd
        ↓
mavlink_imu_bridge.py
        ↓
MAVLink ODOMETRY
        ↓
PX4
        ↓
vehicle_visual_odometry
```

`test_odometry.py` публикует тест:

```text
position = [1.23, 0.0, 0.0]
orientation = [1, 0, 0, 0]
linear velocity = 0
angular velocity = 0
rate = 30 Hz
```

Frames:

```text
pose frame:     MAV_FRAME_LOCAL_FRD
velocity frame: MAV_FRAME_BODY_FRD
estimator:      MAV_ESTIMATOR_TYPE_VIO
```

Финальная проверка transport rate:

```text
IMU RX   ≈ +500 / 5 s  -> ~100 Hz
ODOM RX  ≈ +150 / 5 s  -> ~30 Hz
ODOM TX  ≈ +150 / 5 s  -> ~30 Hz
TX ERR   = 0
```

Пример:

```text
Traffic: IMU RX=15522 (+502) ODOM RX=2852 (+150) ODOM TX=2852 (+150) TX ERR=0
```

`listener vehicle_visual_odometry 5` на PX4 показывает корректную тестовую позицию.

External vision пока НЕ включён в EKF2.

---

## 11. Ground-link monitor

Добавлен:

```text
scripts/wifi_link_monitor.py
```

Ground laptop отправляет UDP heartbeat:

```text
10 Hz
UDP port 15050
prefix: VIO_HOMING_HEARTBEAT
```

Jetson публикует:

```text
/vio_homing/link_state
/vio_homing/link_alive
/vio_homing/link_age
/vio_homing/link_packet_count
```

Логика:

```text
heartbeat age < 0.5 s  -> LINK_OK
0.5 ... 3.0 s         -> LINK_DEGRADED
>= 3.0 s               -> LINK_LOST
до первого пакета      -> WAITING
```

Переходы `WAITING → LINK_OK → LINK_DEGRADED → LINK_LOST → LINK_OK` проверены.

При heartbeat 10 Hz `link_age` обычно находится около `0...0.05 s`, иногда около `0.1 s`, что соответствует ожидаемой работе.

Сейчас тест проводится через Ethernet. Wi-Fi-интерфейс на Jetson отсутствует, поэтому реальный AP пока не настроен.

---

## 12. Trajectory recorder и return path

Добавлен:

```text
scripts/trajectory_recorder.py
```

Входы:

```text
/vio/odom_frd
/vio_homing/link_state
```

Выходы:

```text
/vio_homing/trajectory
/vio_homing/trajectory_point_count
/vio_homing/trajectory_status
/vio_homing/last_good_link_pose
/vio_homing/return_path
/vio_homing/return_ready
```

Логика:

```text
LINK_OK
  -> пишем траекторию
  -> обновляем last_good_link_point

LINK_DEGRADED
  -> продолжаем писать
  -> last_good_link_point больше не обновляется

LINK_LOST
  -> строим обратный сегмент
  -> return_ready=True

LINK_OK после восстановления
  -> старый return_path очищается
  -> return_ready=False
```

Проверенный тест:

```text
LINK_OK
→ LINK_DEGRADED через ~0.5 s
→ LINK_LOST через ~3 s
→ RETURN PATH READY
```

Пример:

```text
RETURN PATH READY: 3 points, target index=202, total trajectory=205
LAST GOOD LINK POINT: x=1.230 y=0.000 z=0.000
```

---

## 13. Автозапуск Jetson

Автозапуск companion stack настроен и проверен после reboot.

### UART

`nvgetty` отключён.

Создано udev-правило:

```text
/etc/udev/rules.d/99-vio-uart.rules
```

```text
KERNEL=="ttyTHS1", GROUP="dialout", MODE="0660"
```

Пользователь `jetson` добавлен в `dialout`.

После reboot:

```text
/dev/ttyTHS1 -> root:dialout
crw-rw----
```

Ручной `chmod 666` больше не нужен.

### systemd

Создан:

```text
/etc/systemd/system/vio-homing.service
```

Сервис:

```text
enabled
active (running)
```

После загрузки Jetson автоматически запускаются:

```text
roslaunch
rosmaster
rosout
mavlink_imu_bridge.py
wifi_link_monitor.py
trajectory_recorder.py
```

Проверенный `rostopic list` после reboot:

```text
/pixhawk/imu
/vio/odom_frd
/vio_homing/last_good_link_pose
/vio_homing/link_age
/vio_homing/link_alive
/vio_homing/link_packet_count
/vio_homing/link_state
/vio_homing/return_path
/vio_homing/return_ready
/vio_homing/trajectory
/vio_homing/trajectory_point_count
/vio_homing/trajectory_status
```

---

## 14. Полный стендовый smoke-test

Успешно проверена цепочка:

```text
Power ON
   ├── Pixhawk
   │     └── PX4
   │          └── extras.txt
   │               └── HIGHRES_IMU ~100 Hz
   │
   └── Jetson
         └── systemd
              └── vio-homing.service
                    ├── rosmaster
                    ├── MAVLink bridge
                    ├── link monitor
                    └── trajectory recorder
```

Далее:

```text
test_odometry.py
      ↓ 30 Hz
/vio/odom_frd
      ↓
MAVLink ODOMETRY
      ↓
PX4 vehicle_visual_odometry
```

И параллельно:

```text
ground heartbeat
      ↓
LINK_OK
      ↓
record trajectory
      ↓
heartbeat stop
      ↓
LINK_DEGRADED
      ↓
LINK_LOST
      ↓
return_path
      ↓
heartbeat restored
      ↓
LINK_OK
      ↓
return state cleared
```

Весь этот стендовый цикл проверен успешно.

---

## 15. Git / репозиторий

Репозиторий:

```text
git@github.com:TheMrKan/vio_homing.git
```

Jetson не всегда имеет интернет. Рабочая резервная схема:

```text
Jetson
  ↓ git bundle
Mac
  ↓ scp / local import
GitHub
```

На Mac настроен SSH key для доступа к приватному репозиторию.

---

## 16. Что закрыто

Готово:

- Ethernet Mac ↔ Jetson;
- SSH;
- ROS Melodic;
- VINS-Fusion собран;
- `vio_homing` оформлен как ROS package;
- UART Pixhawk ↔ Jetson;
- MAVLink 115200;
- автоматические MAVLink streams на PX4;
- `HIGHRES_IMU` ~100 Hz;
- MAVLink → ROS IMU;
- TIMESYNC;
- проверка IMU timing;
- ROS Odometry → MAVLink ODOMETRY;
- `vehicle_visual_odometry` принимается PX4;
- ODOM RX ≈ ODOM TX ≈ 30 Hz;
- TX errors = 0;
- UDP heartbeat monitor;
- состояния link monitor;
- запись траектории;
- last good link point;
- генерация reverse return path;
- очистка return state после восстановления связи;
- постоянные права `/dev/ttyTHS1`;
- отключение `nvgetty`;
- systemd autostart;
- автоматический старт ROS stack после reboot;
- полный стендовый smoke-test.

---

## 17. Ближайшие задачи

### 1. Синтетический движущийся маршрут

Перед камерой проверить recorder на изменяющейся odometry:

```text
X: 0 -> 20+ m
```

После потери heartbeat проверить, что return path действительно идёт назад:

```text
20 -> 19 -> 18 -> ... -> last_good_link_point
```

### 2. Wi-Fi AP на Jetson

После установки совместимого Wi-Fi-адаптера:

- проверить `AP` mode;
- поднять постоянный SSID;
- задать статический адрес;
- включить autoconnect;
- перенести heartbeat с Ethernet на Wi-Fi;
- провести реальный тест потери связи.

### 3. USB-камера

- определить `/dev/video*`;
- поднять ROS image topic;
- проверить FPS и timestamp;
- intrinsic calibration.

### 4. Camera ↔ IMU extrinsics

Получить взаимное положение и ориентацию камеры и IMU/Pixhawk.

### 5. Реальный VINS-Fusion

Входы:

```text
/pixhawk/imu
camera image topic
```

Выход:

```text
VIO pose / odometry
```

### 6. VINS → PX4 FRD

Явно реализовать и проверить преобразование координат/ориентации.

### 7. EKF2 external vision

```text
VINS
  ↓
ROS Odometry
  ↓
MAVLink ODOMETRY
  ↓
vehicle_visual_odometry
  ↓
EKF2
```

### 8. Homing manager

Целевая логика:

```text
NORMAL
  ↓
LINK_LOST / GNSS loss
  ↓
HOMING
  ↓
reverse return_path
  ↓
last_good_link_point
  ↓
WAIT_LINK >= 60 s
  ↓
LAND
```

### 9. Финальный MAVLink stream set

Определить минимальный набор и частоты для:

```text
IMU
attitude
local position
GPS/global position
system state
estimator state
battery
```

с учётом пропускной способности `115200`.

---

## 18. Текущий итог

На текущем этапе автоматически стартует и работает базовый companion stack:

```text
Pixhawk
  │
  ├── HIGHRES_IMU ~100 Hz
  ▼
Jetson
  │
  ├── TIMESYNC
  ├── ROS /pixhawk/imu
  ├── link monitor
  ├── trajectory recorder
  └── return-path generator
```

Обратный канал проверен:

```text
ROS /vio/odom_frd ~30 Hz
        ↓
MAVLink ODOMETRY
        ↓
PX4 vehicle_visual_odometry
```

Главный незакрытый блок — реальная камера и VINS-Fusion. До их подключения инфраструктура ROS/MAVLink transport, времени, детектирования потери связи, записи маршрута, return path и автозапуска companion computer уже проверена на стенде.


## Стенд homing → PX4

Полный отчёт и команды запуска: PX4_HOMING_BENCH.md; логи: verification/px4_bench/. Камеры, GPS и силовой части нет; Wi-Fi не используется. Все движения синтетические, приём MAVLink и ACK проверены на настоящем Pixhawk. ARM не выполнялся, параметры PX4 не менялись.

## Результат зафиксированного прогона

- Полный сценарий PASS; WAIT_LINK до ACK LAND: **60.029621 s**. Переходы ROS: WAIT_LINK 1791044498.614805 → LAND 1791044558.615075 (60.000270 s).
- LAND: один COMMAND_LONG21, MAV_RESULT_ACCEPTED (0); uORB vehicle_command: source_system245, source_component191, from_external=True. vehicle_command_ack: command21, result0. PX4 перешёл в nav_state18 AUTO_LAND, оставаясь arming_state1 и armed_time0. Затем исходный nav_state4 восстановлен.
- Каждая уникальная следующая цель FRD x=3,2,1,0 подтверждена на PX4 как NED (10,-17,2), (10,-18,2), (10,-19,2), (10,-20,2), в двух сценариях.
- IMU **99.55 Hz**, ODOMETRY **29.93 Hz**, одометрия отдельно прочитана в vehicle_visual_odometry. После завершения исходный companion bridge возобновлён: IMU около100 Hz.
- Отмена RETURN, отмена WAIT_LINK с наблюдением61 s, stale odometry, пустой маршрут/отсутствующая odometry, реальное закрытие MAVLink и переподключение, повторный LAND, перезапуск homing_manager: PASS.
- Catkin: **24 tests, 0 errors, 0 failures, 0 skipped**; включая20 чистых тестов Python2.7 и ROS-интерфейсы/сценарий.

На стенде Jetson Nano + Pixhawk 6C Mini подтверждён программный тракт от детекции потери связи и формирования обратной траектории до передачи команд возврата и LAND в PX4. Реальное выполнение команд БВС будет проверяться после сборки аппарата.


## Объединение с GitHub / изменения коллеги

GitHub main4c645ce (MrKan: USB camera node, camera timing, TASK.md и PEP formatting) объединён с проверенным Jetson767f111 в отдельной ветке integrate/jetson-github. Merge commit9c5e2ef содержит обе истории. Конфликты bridge/launch/CMake разрешены; семантика существующих узлов не изменилась, что подтверждено AST. Camera и cv_bridge добавлены, enable_camera=false до физического подключения. Все исходные файлы и незакоммиченные правки сохранены в отдельной папке jetson/backup и stash.

Повторная сборка и24 catkin tests прошли; повторный аппаратный bench PASS: WAIT_LINK 60.145384s до ACK LAND MAV_RESULT_ACCEPTED; IMU99.99Hz / ODOMETRY30.00Hz. Все cancellation/freshness/restart/disconnect проверки прошли. Логи verification/github_merge/. ARM и изменения параметров PX4 не выполнялись, камера отсутствует, Wi-Fi не используется. Push не выполнялся. Подробности MERGE_REPORT.md.


## Единый стиль и ограниченные оптимизации

Все19 Python-файлов оформлены по PEP8 (79 символов,4 пробела) с помощью YAPF; добавлены setup.cfg, .editorconfig и requirements-format.txt. Launch/XML оформлены единообразно. Форматирование сохраняет структуру кода.

Оптимизации: предварительный cos/sin для фиксированного FRD→NED преобразования, один список вместо двух при reverse return_path, JSON tx_event создаётся только при наличии подписчиков. ROS/MAVLink интерфейсы, оси, таймауты и LAND gates сохраняются. Подробности DEVELOPMENT.md. По просьбе владельца функциональные/ROS/PX4 tests не запускались; требуется только пересборка на Jetson.

Пересборка на Jetson выполнена:catkin_make -j2, exit0. Лог и отчёт:verification/formatting/. Функциональные/ROS/PX4 тесты в этом этапе не запускались.

## 2026-10-17 — локальная подготовка USBGS1200P01

- Добавлены отдельные camera.launch и record_camera_imu.launch без запуска UART.
- YAML intrinsics проверяется по размерам, структуре, модели, конечности коэффициентов и focal lengths. При ошибке явно указанного файла запуск останавливается. При фактическом несовпадении размера кадры не публикуются.
- Camera image frame уточнён как camera_optical_frame; extrinsics к IMU не заданы.
- Добавлены регрессионные тесты калибровки и CAMERA_BRINGUP.md с процедурами шахматной доски, AprilGrid/Kalibr и ограничениями подключения VINS.
- Локально Python 3: 24 unit tests PASS (20 существующих, 4 новых). XML launch/package разбирается. Это не catkin/ROS/Python 2.7 или аппаратный тест.
- На Jetson ничего не перенесено и не перезапущено; калибровочных измерений ещё нет.
