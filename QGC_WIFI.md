# QGroundControl через AP Jetson

Mac подключается к AP VIO-HOMING. Pixhawk подключён к Jetson UART TELEM2,
460800. Единственный владелец UART — mavlink_imu_bridge.py.

Во втором терминале:

```bash
VIO_QGC_IP=192.168.50.213 ~/catkin_ws/src/vio_homing/scripts/manual/02_px4.sh
```

В QGC включить UDP link с локальным портом 14550. Приём на Jetson:
192.168.50.1:14555. При изменении Wi-Fi IP Mac изменить VIO_QGC_IP.
Мост пересылает MAVLink штатно и учитывает heartbeat sysid=255, compid=190,
type=GCS. Новый peer автоматически не подхватывается.

03_logic.sh запускает монитор heartbeat: LINK_OK → LINK_DEGRADED → LINK_LOST
при порогах 2/4 секунды. Восстановление связи отменяет RETURN/WAIT_LINK.

Полная последовательность пяти ручных терминалов и отдельного задания origin:
MANUAL_START.md. Mission требует пригодной глобальной позиции и миссии;
задание origin не гарантирует готовность EKF2 или пригодность миссии.
Возврат проекта использует текущие локальные цели OFFBOARD, не PX4 Mission.
