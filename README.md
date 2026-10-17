# VIO Homing

ROS Melodic / Python 2.7 companion software for Jetson Nano and PX4.

На стенде Jetson Nano + Pixhawk 6C Mini подтверждён программный тракт от детекции потери связи и формирования обратной траектории до передачи команд возврата и LAND в PX4.

## Что реализовано

- Pixhawk IMU → ROS около100 Гц, TIMESYNC.
- ROS Odometry → MAVLink ODOMETRY → PX4 около30 Гц.
- Heartbeat LINK_OK → LINK_DEGRADED → LINK_LOST.
- Запись траектории, last_good_link_pose и обратный return_path.
- homing_manager: последовательные цели → WAIT_LINK ≥60s → LAND; восстановление heartbeat отменяет возврат/ожидающий LAND.
- Текущая цель передаётся отдельным SET_POSITION_TARGET_LOCAL_NED; LAND — COMMAND_LONG/MAV_CMD_NAV_LAND с реальным ACK от PX4.
- Узел USB-камеры и проверка частоты кадров; камера выключена по умолчанию до подключения.

ODOMETRY и SETPOINT — отдельные интерфейсы. Привязка LOCAL_FRD → NED задана явно; тестовая synthetic-привязка не является калибровкой реального VIO. Команды homing по умолчанию выключены до подтверждения привязки.

## Текущий стенд

Mac —Ethernet→ Jetson Nano —UART/TELEM2→ Pixhawk 6C Mini; Pixhawk также подключён к Mac по USB. Камеры, GPS, моторов, ESC и пропеллеров нет; Wi-Fi не используется. Движение и odometry синтетические; приём команд, uORB и ACK проверены на настоящем Pixhawk без ARM и изменения защит.

PX4:1.16.0, git `6ea3539157ca358c70a515878b77077af7d4611d`. После объединения с изменениями коллеги повторный bench прошёл: WAIT_LINK до ACK LAND60.145s, MAV_RESULT_ACCEPTED; IMU100Hz, ODOMETRY30Hz. 24 catkin tests без ошибок.

Реальное выполнение возврата и посадки будет проверяться после сборки БВС. Камера/VINS и реальная привязка к frame EKF пока не проверены.

## Запуск на Jetson

Пакет размещается в `~/catkin_ws/src/vio_homing`.

```bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make -j2
catkin_make run_tests_vio_homing -j1
catkin_test_results build/test_results/vio_homing
```

Повтор аппаратного bench при уже работающем companion bridge:

```bash
rosrun vio_homing run_px4_homing_bench.py
cat /tmp/vio_homing_px4_bench/latest.json
```

Runner временно приостанавливает штатный bridge для исключения двух читателей UART и возобновляет его после теста. Synthetic bench использует loopback UDP, не Wi-Fi. Ожидается `passed: true`, `land_tx: 1`, `wait_link_seconds >= 60` и ACK от Pixhawk.

## Документация и логи

- [Прогресс проекта](VIO_HOMING_PROGRESS.md).
- [PX4 bench: сообщения, координаты, ROS topics и команды](PX4_HOMING_BENCH.md).
- [Объединение Jetson и GitHub, изменения коллеги](MERGE_REPORT.md).
- [Повторная проверка объединённой версии](verification/github_merge/result.json), [PX4/uORB console](verification/github_merge/console.log).
- [Техническое задание](TASK.md), [окружение](ENVIRONMENT.md).

История изменений коллеги и стендовых этапов сохранена в Git.


## Обслуживание кода

[Единый стиль, инструменты и ограниченные оптимизации](DEVELOPMENT.md). После последнего форматирования/оптимизаций функциональные тесты не запускались по просьбе владельца; предыдущие результаты сохранены как история.

## Подключение камеры (подготовка 2026-10-17)

[USBGS1200P01: запуск, калибровка и запись camera+IMU](CAMERA_BRINGUP.md).
Точка доступа VIO-HOMING работает по выводу владельца; реальный heartbeat по Wi-Fi пока не проверен.
Новая локальная подготовка ещё не перенесена на Jetson.
