# VIO Homing — рабочий пакет Jetson

ROS Melodic / Python 2.7, Jetson Nano, Pixhawk 6C Mini (PX4 1.16.0).

## Ручной запуск

Автозапуск отключён. См. [MANUAL_START.md](MANUAL_START.md).
Пять скриптов в `scripts/manual/` запускаются в отдельных терминалах.
Браузерное видео, панель и старые средства автозапуска удалены.
В шестом терминале origin задаётся явно; команды в MANUAL_START.md.

## Наблюдение по SSH

```bash
source ~/catkin_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.50.1 ROS_MASTER_URI=http://localhost:11311
rosrun vio_homing vins_displacement.py
```

`r` + Enter — новый ноль индикатора. Обнуление не исправляет VINS и не снимает
FAULT адаптера. См. VINS_DISPLAY.md.

```bash
rostopic echo -n 1 /vio_homing/vins_adapter_status
rostopic echo -n 1 /vio_homing/mavlink_transport_status
```

## Состав

- `scripts/`: рабочие ROS узлы, индикатор перемещения, проверки IMU/камеры.
- `src/vio_homing/`: координаты, контроль данных, логика возврата и MAVLink.
- `launch/`: отдельные PX4, логика, камера, VINS/адаптер и запись camera–IMU.
- `config/`: параметры VINS/камеры, справочный профиль PX4.

## Интерфейсы

| Назначение | Значение |
|---|---|
| Jetson AP/SSH | 192.168.50.1 |
| Mac / QGC | 192.168.50.213 (текущий), UDP 14550; при смене адреса обновить peer |
| Приём QGC на Jetson | UDP 14555 |
| UART | Jetson /dev/ttyTHS1 → PX4 /dev/ttyS3, 460800 |
| IMU | /pixhawk/imu, около 200 Гц |
| Камера | /camera/image_raw, 640×480 YUYV, около 30 Гц |
| VINS | /vins_estimator/odometry |
| Одометрия для PX4/возврата | /vio/odom_frd |
| Обратный маршрут / цель | /vio_homing/return_path / /vio_homing/return_goal |

ODOMETRY передаёт оценку положения; SET_POSITION_TARGET_LOCAL_NED — текущую цель.
Путь остаётся на Jetson. Возврат: RETURN → WAIT_LINK ≥60 с → LAND; восстановление
GCS отменяет RETURN/WAIT_LINK. Flight-контроллер не ARMит аппарат и не выдаёт
управление в disarmed/наземном состоянии.

Документы: SAFETY_BEHAVIOR.md (отказы и ограничения), AUTONOMOUS_HOMING.md (EKF2/координаты/управление), VINS_SETUP.md
(калибровка), WIRELESS_SETUP.md, QGC_WIFI.md, CAMERA_BRINGUP.md, TASK.md.

## Очистка

21.10.2026 тестовые/synthetic сценарии и исторические verification-артефакты
удалены из рабочего дерева по запросу владельца. Резервная копия и список
удалений указаны в CLEANUP.md. Калибровочные bag/YAML/PDF сохранены.
Старые документы прогресса описывают исторические этапы, включая удалённые тесты.

Работа VINS и минимальный дрейф на месте подтверждены владельцем для текущей
сессии. Прежние сессии имели расхождение; точность, масштаб и поведение на всём
маршруте этим не подтверждены. Реальное выполнение OFFBOARD/возврата/LAND на
полноценном аппарате не подтверждено. Система не объявляется готовой к полёту.
