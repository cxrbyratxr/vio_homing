# Проверка интерфейса homing → PX4 на стенде

Стенд: Jetson Nano, Pixhawk 6C Mini, Mac. UART/TELEM2 между Jetson и Pixhawk; Ethernet между Mac и Jetson. Камеры, GPS, силовой части и БВС нет. Wi-Fi не используется. Одометрия и движение синтетические. Приём MAVLink, uORB и ACK проверяются на настоящем Pixhawk, без ARM и без изменения параметров PX4.

## Интерфейсы

| ROS topic | Тип | Назначение |
|---|---|---|
| `/vio/odom_frd` | nav_msgs/Odometry | Текущая измеренная/синтетическая позиция, local_frd / body_frd |
| `/vio_homing/link_state` | std_msgs/String | WAITING / LINK_OK / LINK_DEGRADED / LINK_LOST |
| `/vio_homing/return_path` | nav_msgs/Path | Обратный маршрут local_frd, хранится на Jetson |
| `/vio_homing/last_good_link_pose` | geometry_msgs/PoseStamped | Конечная точка возврата local_frd |
| `/vio_homing/return_goal` | geometry_msgs/PoseStamped | Только очередная цель local_frd |
| `/vio_homing/homing_state` | std_msgs/String | NORMAL / LINK_LOST / RETURN / WAIT_LINK / LAND |
| `/vio_homing/return_active`, `/vio_homing/land_command` | std_msgs/Bool | Управление транспортом |
| `/vio_homing/homing_session` | std_msgs/String | UUID экземпляра менеджера; новый запуск очищает старые намерения |
| `/vio_homing/mavlink_transport_status` | std_msgs/String (JSON) | Счётчики, свежесть связи, TIMESYNC, результат LAND |
| `/vio_homing/mavlink_tx_event`, `/vio_homing/mavlink_command_ack` | std_msgs/String (JSON) | Передача и ACK от PX4 |
| `/vio_homing/px4_console_request`, `/vio_homing/px4_console_response` | std_msgs/String (JSON) | Только в bench launch: чтение MAVLink Console |
| `/vio_homing/bench_control` | std_msgs/String (JSON) | Только bench: disarmed OFFBOARD, восстановление исходного режима, временное закрытие UART |

Один bridge владеет одним UART-соединением. ODOMETRY сообщает текущую позицию (MAVLink ODOMETRY, LOCAL_FRD/BODY_FRD); отдельный SET_POSITION_TARGET_LOCAL_NED (84), LOCAL_NED (1), mask 3576 передаёт только позиционную цель примерно 20 Гц. Игнорируются скорость, ускорение, yaw и yaw rate. В WAIT_LINK продолжается передача последней цели. В LAND отправляется COMMAND_LONG / MAV_CMD_NAV_LAND (21), один раз за эпизод; координаты и yaw NaN означают текущую позицию/не задано. COMMAND_ACK публикуется отдельно, результат отказа не обходится. Весь маршрут не отправляется как Mission.

Транспорт требует свежих целей/состояния (0.5 s), одометрии (1 s по stamp и времени получения), heartbeat PX4, converged TIMESYNC для setpoint, UUID менеджера и явно подтверждённой привязки координат. При потере MAVLink команды прекращаются; старый LAND после переподключения не воспроизводится. Повтор LAND не повторяет COMMAND_LONG. Восстановление heartbeat оператора отменяет RETURN и таймер WAIT_LINK. Для данного этапа транспорт дополнительно разрешён только при подтверждённом disarmed PX4.

## Координаты

LOCAL_FRD не равен NED: это фиксированный локальный frame с произвольным курсом и Z вниз. Для точки `(x,y,z)`:

```
N = origin_N + cos(psi)*x - sin(psi)*y
E = origin_E + sin(psi)*x + cos(psi)*y
D = origin_D + z
```

Для synthetic bench явно заданы `psi=90°`, `origin_NED=(10,-20,2)`. Например FRD `(3,0,0)` → NED `(10,-17,2)`. Это тестовая привязка, не калибровка реального VIO. Преобразований ENU/FLU здесь нет: вход уже FRD. Для реального VIO потребуется отдельно измерить курс/смещение и согласованность с локальным frame EKF PX4. По умолчанию companion.launch имеет `enable_homing_commands=false` и `frd_to_ned_alignment_valid=false`.

## PX4 и способ проверки

`ver all`: PX4 1.16.0 Release, git `6ea3539157ca358c70a515878b77077af7d4611d`, PX4_FMU_V6C / V6C000002, сборка Aug 6 2025 15:07:26.

Реализация этой версии: [mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/6ea3539157ca358c70a515878b77077af7d4611d/src/modules/mavlink/mavlink_receiver.cpp). Приём LOCAL_NED публикует offboard_control_mode; численная trajectory_setpoint публикуется только в OFFBOARD. Поэтому тест предварительно передаёт цели, запрашивает OFFBOARD через COMMAND_LONG176 и проверяет ACK. На данном Pixhawk этот переход разрешён в disarmed; arming_state=1, armed_time=0 проверяются через vehicle_status. Исходный режим восстанавливается после сценария. Если на другой прошивке переход отклонён, тест завершается с отказом и не отключает защиты.

Проверки PX4 через SERIAL_CONTROL/MAVLink Console на том же UART:

- `listener offboard_control_mode 1`: position=True, velocity=False, acceleration=False.
- `listener trajectory_setpoint 1`: численные значения каждой следующей цели, сравнение с преобразованием до 1e-4.
- `listener vehicle_visual_odometry 1`: отдельно проверяется приём исходной FRD одометрии.
- `listener vehicle_command 1`, `listener vehicle_command_ack 1`: LAND command21 и обработка.
- MAVLink COMMAND_ACK21 от Pixhawk; `listener vehicle_status 1`: отсутствие ARM.

Аудит прочитал MAV_1_MODE=1, MAV_1_FORWARD=0, EKF2_EV_CTRL=0, COM_OF_LOSS_T=1, COM_OBL_RC_ACT=0. Параметры не менялись. EKF2_EV_CTRL=0 означает, что приём vehicle_visual_odometry не доказывает использование VIO в оценке позиции. Нет валидного XY и подтверждённого навигационного курса; физическое выполнение возврата/посадки не проверяется.

## Запуск на Jetson

В действующей SSH-сессии:

```bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make -j2
catkin_make run_tests_vio_homing -j1
catkin_test_results build/test_results/vio_homing
rosrun vio_homing run_px4_homing_bench.py
cat /tmp/vio_homing_px4_bench/latest.json
cat /tmp/vio_homing_px4_bench_launch.log
```

Runner ожидает один уже работающий companion bridge, временно приостанавливает его, запускает изолированный ROS master11331 и всегда возобновляет старый bridge при завершении. На Jetson UART `/dev/ttyTHS1`; название `/dev/ttyS3` из `mavlink status` относится к стороне PX4. Не запускайте два bridge одновременно на одном UART. Временная приостановка исключает конкурирующих читателей; новый bench bridge продолжает проверку IMU/ODOMETRY.

Ожидается `passed: true`, `land_tx: 1`, `wait_link_seconds >= 60`, реальные uORB-позиции и ACK21. Тест занимает несколько минут: восстановление в RETURN, stale odometry, реальное закрытие MAVLink, отмена WAIT_LINK с наблюдением 61 s после восстановления, полный маршрут и отдельное ожидание ≥60 s до LAND, повторный LAND, перезапуск менеджера. Пустой маршрут/отсутствующая одометрия проверяются до первого движения. Дополнительные чистые Python2 тесты покрывают оси/поворот/смещение, stale manager/target, NaN/frame, TIMESYNC, перезапуски и запрет повторного LAND.

Полный результат и логи данного прогона находятся в `verification/px4_bench/`. Логи относятся к часам Jetson; его системные часы не корректировались в ходе теста.

## Ограничения

Подтверждается программный тракт и значения внутри PX4, а не движение аппарата. Камера/VINS и реальная привязка VIO к NED ещё не проверены. Heartbeat создаётся loopback UDP, не радио/Wi-Fi. LAND ACK означает обработку команды, не реальную посадку. Контроль движения, оценка положения, поведение с силовой частью и полётные failsafe требуют будущих отдельных испытаний после сборки БВС.

## Результат зафиксированного прогона

- Полный сценарий PASS; WAIT_LINK до ACK LAND: **60.029621 s**. Переходы ROS: WAIT_LINK 1791044498.614805 → LAND 1791044558.615075 (60.000270 s).
- LAND: один COMMAND_LONG21, MAV_RESULT_ACCEPTED (0); uORB vehicle_command: source_system245, source_component191, from_external=True. vehicle_command_ack: command21, result0. PX4 перешёл в nav_state18 AUTO_LAND, оставаясь arming_state1 и armed_time0. Затем исходный nav_state4 восстановлен.
- Каждая уникальная следующая цель FRD x=3,2,1,0 подтверждена на PX4 как NED (10,-17,2), (10,-18,2), (10,-19,2), (10,-20,2), в двух сценариях.
- IMU **99.55 Hz**, ODOMETRY **29.93 Hz**, одометрия отдельно прочитана в vehicle_visual_odometry. После завершения исходный companion bridge возобновлён: IMU около100 Hz.
- Отмена RETURN, отмена WAIT_LINK с наблюдением61 s, stale odometry, пустой маршрут/отсутствующая odometry, реальное закрытие MAVLink и переподключение, повторный LAND, перезапуск homing_manager: PASS.
- Catkin: **24 tests, 0 errors, 0 failures, 0 skipped**; включая20 чистых тестов Python2.7 и ROS-интерфейсы/сценарий.

На стенде Jetson Nano + Pixhawk 6C Mini подтверждён программный тракт от детекции потери связи и формирования обратной траектории до передачи команд возврата и LAND в PX4. Реальное выполнение команд БВС будет проверяться после сборки аппарата.

## Изменённые и добавленные файлы

- Менеджер: `scripts/homing_manager.py`, `src/vio_homing/homing_logic.py`, `src/vio_homing/__init__.py`.
- MAVLink/координаты/защита повторов: `scripts/mavlink_imu_bridge.py`, `src/vio_homing/homing_transport.py`, `src/vio_homing/mavlink_homing_adapter.py`.
- Стенд: `scripts/homing_stand_sim.py`, `scripts/px4_homing_bench.py`, `scripts/run_px4_homing_bench.py`, `launch/homing_stand.launch`, `launch/px4_homing_bench.launch`.
- Запуск/сборка: `launch/companion.launch`, `CMakeLists.txt`, `package.xml`, `setup.py`, `.gitignore`.
- Тесты: `test/test_homing_logic.py`, `test/test_homing_transport.py`, `test/homing_interfaces.py`, `test/homing_interfaces.test`, `test/homing_stand.test`.
- Документация: `VIO_HOMING_PROGRESS.md`, `HOMING_STAND.md`, `HOMING_VERIFICATION.md`, этот отчёт; доказательства `verification/px4_bench/`.

`wifi_link_monitor.py` и `trajectory_recorder.py` не изменены. Пользовательские изменения `check_imu_timing.py`, `TASK.md` и backup bridge не включены в commit. IMU/TIMESYNC/ODOMETRY сохранены. Push не выполнялся.
