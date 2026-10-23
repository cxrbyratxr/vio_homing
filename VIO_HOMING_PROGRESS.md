# VIO Homing — текущий прогресс

Обновлено: 17.10.2026

Документ фиксирует текущее состояние стенда и выполненные работы по интеграции Jetson Nano, Pixhawk 6C Mini, PX4, ROS Melodic и VINS-Fusion.

## Актуальный итог на 17.10.2026

Текущая схема: Mac по Wi-Fi AP VIO-HOMING ↔ Jetson Nano ↔ UART460800 ↔
Pixhawk 6C Mini. USBGS1200P01 подключена: цветное изображение640×480,30Гц.
QGC и браузер работают через AP; IMU около200Гц, TIMESYNC. Владелец сообщил об
успешной работе после установки vio-wireless.service. Холодный старт питания
ещё отдельно не подтверждён. Kalibr собран, AprilGrid подготовлена, фокус
зафиксирован; запись и калибровка ещё не выполнены. Реальная VIO и GPS-триггер
не подключены, аппарат не собран. Полётный homing не реализован: текущий
адаптер для разоружённого стенда, команды в wireless-запуске выключены.

Краткая сводка: [PROJECT_STATUS.md](PROJECT_STATUS.md). Текущая инструкция:
[WIRELESS_SETUP.md](WIRELESS_SETUP.md). Разделы ниже — история этапов;
формулировки «камеры нет», «Wi-Fi не используется», «ещё не перенесено» и старые
частоты/скорости описывают соответствующий прошлый этап, не текущее состояние.

## История разработки

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

## 2026-10-17 — локальный MAVLink relay QGC/AP

- Опциональный fixed-peer UDP relay встроен в единственный UART bridge. Исходные MAVLink bytes/IDs сохраняются, PX4 telemetry/ACK отправляются обратно QGC.
- Добавлен режим monitor source=qgc и qgc_wifi.launch: heartbeat QGC через Wi-Fi, OK<2s, DEGRADED<4s, LOST>=4s. Старый UDP monitor по умолчанию сохранён.
- Peer IP и GCS IDs фильтруются; команды требуют свежего GCS heartbeat. Reconnect удаляет очередь UDP и heartbeat. Это не криптографическая аутентификация.
- Локально 29 unit tests PASS; пять новых используют поддельный socket/parser без сетевого обмена. Настоящий pymavlink, ROS/Python2.7 и аппаратура не проверены в этом этапе.
- На Jetson не переносилось. Полётный возврат не добавлен, disarmed gate сохранён. QGC channel не означает готовность ARM/Takeoff/движения или failsafe в полёте. См. QGC_WIFI.md.

## 2026-10-17 — исправление частоты камеры

Парный тест владельца на Jetson, OpenCV V4L2, YUYV 640x480, запрос30Hz:
без настройки BUFFERSIZE:120 кадров,28.87Hz,grab33.31ms,retrieve1.32ms;
BUFFERSIZE=1:120 кадров,15.22Hz,grab63.72ms,retrieve1.99ms.
Удалена принудительная настройка одного буфера в usb_camera_node.py.
Частота полного ROS узла после исправления ещё требует проверки на Jetson.
Это измерение частоты, не калибровка timestamps/экспозиции или latency.

## 2026-10-17 — integrated wireless trial launch

wireless.launch + run_wireless.sh combine AP ROS addressing, QGC fixed-peer relay,
color YUYV640x480@30 preview and single UART460800. Request HIGHRES_IMU200 on
each connection with COMMAND_LONG/SET_MESSAGE_INTERVAL; log ACK and validate
UART headroom. Existing launch defaults remain compatible. Commands for homing
remain disabled. Stop old service/processes before update/start.
29 existing local Python3 tests PASS; XML wiring and source syntax checked.
New combined runtime/ROS Python2 integration still requires Jetson test.
Measured owner results for camera30Hz and IMU196.17Hz are historical component
checks, not proof of the new launch. See WIRELESS_SETUP.md.

## 2026-10-17 — systemd wireless autostart preparation

Added vio-wireless.service + installer:separate service, old service preserved
and disabled on installation, group-wide shutdown, restart on missing AP/failure,
fixed current AP/Mac IP, independent of SSH. Installer rejects live ROS processes
and saves previous unit/state. AP autoconnect is an explicit operator step.
Local bash syntax/unit structural checks passed; actual systemd/boot behavior
must be verified on Jetson. No claims of flight readiness. See systemd/README.md.

## 2026-10-21 — IMU rate corrected at PX4 source

Direct UART reading reproduced~147Hz without ROS/camera. PX4 integration200Hz
and HIGHRES_IMU scheduler200Hz did not deliver the requested packet cadence.
Raised only IMU_INTEG_RATE200→400, saved and rebooted Pixhawk; UART460800 and
MAVLink request200Hz unchanged. Direct UART measured199.36Hz afterward.
No fabricated/upsampled ROS messages. Details and full-stack measurement:
[verification/imu_rate/README.md](verification/imu_rate/README.md).
Odometry,calibration,EKF and failsafe parameters unchanged.

Минутная проверка полного стека после исправления: IMU199.5467Гц, камера30.0034Гц,
QGC LINK_OK,TIMESYNC OK,нет обратных меток IMU и интервалов>15мс; max9.043мс.
Подробности и исходный JSON сохранены в verification/imu_rate/.


## 2026-10-21 — VINS autostart, EKF2 and OFFBOARD controller

Added VINS+adapter to wireless service, fixed FRD→NED yaw/origin registration
from matched VINS/PX4 estimator poses while disarmed. Added gated OFFBOARD
prestream/mode confirmation, position-only current targets, recovery to Hold,
one-shot LAND and bounded failure/restart handling. No ARM commands.
Owner explicitly approved EKF2 EV profile and GCS/RC/OFFBOARD failsafe changes;
readback after PX4 reboot confirmed persistence. Full parameter backups and
build/runtime evidence in verification/control_stage; operational details and
limitations in AUTONOMOUS_HOMING.md. Jetson package built; 41 Python2 tests passed.
Legacy pymavlink ODOMETRY signature handled explicitly so quality cannot be
misinterpreted as force_mavlink1. Quality=0 means unknown; no fabricated quality.
Live flight control remains unverified; no claim of flight readiness or drift-free VIO.
Live integration caveat: 168 ODOMETRY packets sent without TX errors, then VINS
position discontinuity latched adapter FAULT. Automatic NED alignment did not
complete; sustained EV fusion and armed control are NOT verified. Keep fault
visible; do not bypass max_step or freshness checks to obtain an alignment.

## 2026-10-23 — ручной запуск, очистка и стендовый origin

Предыдущие разделы — история; актуальный порядок в MANUAL_START.md.
Удалены systemd templates/installer, run_wireless.sh, общие companion/wireless/
qgc_wifi launch, панель и запуск браузерного видео. Взамен scripts/manual/01–05.
ROS master явный; дубликаты проверяются, respawn и MAVLink auto-reconnect по
умолчанию отключены. AP/SSH остаются сетевыми службами ОС.
В шестом терминале set_bench_origin.py задаёт условную привязку МАИ 4к3;
PX4 подтвердил её через ref_lat/ref_lon/ref_alt, xy_global/z_global.
Механизм не посылает GPS/HIL_GPS, не ARMит и не переключает режимы.

Перенесено g_norm=9.807 из фактического конфига Jetson. Добавлена блокировка
сессии при разрыве IMU/сбросе её времени; до сходимости TIMESYNC нет IMU.
Скачок VINS >1 м по просьбе владельца переведён в предупреждение, остальные
проверки сохранены. Имелись реальные срывы VINS и отказы EKF2; результат
текущего порядка запуска сообщил владелец. Это не доказательство устойчивости
VIO, выполнения полётных команд или готовности к полёту.

## 2026-10-23 — обработка отказов и готовность возврата

Добавлены VioGuard (скорость/ускорение/шаг + 3 с стабилизации), подтверждение
прибытия по расстоянию/скорости/выдержке, тайм-аут продвижения и ABORT→LAND.
Смена навигационной эпохи очищает старый маршрут. Перерегистрация после
завершения магнитного yaw alignment допускается в NORMAL до захвата возврата;
reset во время возврата — отказ. После регистрации контролируется расхождение.

По исходнику PX4 1.16 уточнено: старый флаг CONST_POS_MODE включает состояние
vehicle_at_rest; heading_good_for_control при mag зависит от окончательного
выставления курса в воздухе. Ранее False на земле трактовался слишком строго.
Прямые поля локальной позиции читаются ограниченным read-only SERIAL_CONTROL
запросом. Опция не обходит проверки и не посылает ARM. Совместный доступ
QGC MAVLink Console к NSH не поддерживается: ограничения в SAFETY_BEHAVIOR.md.
Профиль дополнительно проверяет бюджет COM_DL_LOSS_T относительно тайм-аутов
нашего захвата; параметры PX4 не изменялись.

На Jetson Python 2.7 выполнены автономные проверки логики и ROS-message I/O
с подменённым транспортом. На реальном Pixhawk диагностический мост с
homing-командами выключенными получил прямой PX4 health; по пятесекундным
счётчикам поток IMU оставался около 200 Гц. Это не проверка полётного управления.
Реальное выполнение возврата/LAND и устойчивость VINS остаются открытыми.

Повторный замер при включённом read-only PX4 health probe: 1000 IMU samples,
199.84 Гц, максимальный dt 0.009038 с, gaps >15 ms = 0, bad timestamps = 0.
Автономные регрессии: 27 сценариев логики + 13 ROS-message проверок на Python 2.7.
