# Подключение камеры и подготовка VIO

Подготовлено локально 2026-10-17. Камера владельца: USBGS1200P01.
На Jetson новая версия ещё не перенесена. Реальная камера, ROS Melodic и
Python 2.7 в этом локальном этапе не проверялись. AP VIO-HOMING подтверждена
выводом владельца: wlan0, 192.168.50.1, 2.4 GHz; heartbeat по Wi-Fi ещё не проверен.

## 1. Проверить устройство на Jetson

Сохранить Ethernet для SSH. Не запускать второй companion.launch поверх
автоматически запущенного: два читателя UART недопустимы.

```bash
source ~/catkin_ws/devel/setup.bash
ls -l /dev/v4l/by-id/
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
rosnode list
rostopic info /camera/image_raw
rostopic info /pixhawk/imu
```

`v4l2-ctl` входит в пакет `v4l-utils`. Выбрать реальное устройство и поддерживаемый
режим; значения 640x480 MJPG 30 в launch — исходные настройки, не подтверждённые
характеристики USBGS1200P01. Предпочтителен постоянный путь /dev/v4l/by-id/.
Не запускать camera.launch, если камера уже включена в companion.

## 2. Камера отдельно от UART

```bash
roslaunch vio_homing camera.launch device:=/dev/video0 width:=640 height:=480 fps:=30 fourcc:=MJPG
```

Ожидается `/camera/image_raw` sensor_msgs/Image mono8, optical frame:
X вправо на изображении, Y вниз, Z вдоль оптической оси вперёд.
Это не IMU FLU и не local_frd. TF между камерой и IMU здесь не выдумывается.
Без YAML CameraInfo не публикуется. Проверить изображение:

```bash
rosrun image_view image_view image:=/camera/image_raw
rostopic hz /camera/image_raw
rosrun vio_homing check_camera_timing.py
```

Просмотр требует графического сеанса. При отсутствии изображения остановить
проверку timing через Ctrl-C: старый checker ожидает 300 кадров без тайм-аута.
Частота кадров >=20 Hz — только входное условие, не доказательство частоты VIO.
Timestamp берётся после grab(), до decode; это НЕ аппаратная метка экспозиции.
Не подбирать time_offset наугад. Зафиксировать режим, фокус и экспозицию.

## 3. Калибровка объектива: шахматная доска

Установить `ros-melodic-camera-calibration` при отсутствии. Измерить размер
клетки в метрах, посчитать ВНУТРЕННИЕ углы. Пример ниже только для 8x6 и 25 mm;
заменить реальными значениями, не использовать без измерения.

```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/camera/image_raw camera:=/camera
```

Показать всю доску в разных частях кадра, под наклоном и на разных расстояниях,
не допускать смаза. CALIBRATE, затем SAVE. Сохранить ost.yaml из выданного
архива в отдельную папку измерений, например ~/vio_data/calibration/camera.yaml.
Узел не предоставляет set_camera_info: использовать SAVE, не рассчитывать на COMMIT.
Перезапустить камеру в том же режиме:

```bash
roslaunch vio_homing camera.launch camera_info_file:=/home/jetson/vio_data/calibration/camera.yaml
rostopic echo -n 1 /camera/camera_info
```

Указать тот же device/width/height/fps/fourcc, что при калибровке.
Сейчас поддерживается ROS YAML plumb_bob с пятью коэффициентами.
Ошибочные, нечисловые коэффициенты и несовпадение разрешения отклоняются.
Фактический размер кадра дополнительно проверяется перед публикацией.
Fisheye/equidistant требует отдельной поддержки; не переименовывать модель.
Оценить остаточные ошибки и прямизну линий на независимых кадрах, сохранить отчёт.

## 4. Camera–IMU: AprilGrid / Kalibr

ROS checkerboard YAML не заменяет калибровку extrinsics/time offset.
Если выбран AprilGrid, нужны реальные tagRows/tagCols/tagSize/tagSpacing,
модель pinhole-radtan или другая по результатам проверки объектива и параметры
шума IMU. Не копировать параметры EuRoC как измерения нашей системы.
Камера и Pixhawk должны быть закреплены жёстко; перемещается вся сборка,
с вращениями вокруг всех осей и поступательными движениями, доска неподвижна.
До записи проверить IMU ~100 Hz и правильность timestamps; подготовить Kalibr
на отдельном совместимом окружении. Не считать наличие Kalibr на Jetson доказанным.

Запись уже существующих publishers (не запускает камеру/bridge):

```bash
mkdir -p ~/vio_data/calibration
roslaunch vio_homing record_camera_imu.launch output_prefix:=/home/jetson/vio_data/calibration/camera_imu
```

Ctrl-C завершает запись. Проверить `rosbag info <файл.bag>`, наличие изображений,
IMU и меток. Split по 1 GiB, суммарный размер не ограничен: контролировать свободное
место. Запись raw mono8 640x480x30 до сжатия ~9.2 MB/s.
Kalibr обычно требует несжатый bag: перед обработкой `rosbag decompress` на копии.
Сохранить исходный bag, описание target, отчёты и camchain/imu YAML.

## 5. Подключение VINS: следующий аппаратный шаг

Сначала сверить установленный форк ~/catkin_ws/src/VINS-Fusion с upstream:
launch, версия, topics, единицы, значение pose/twist. Upstream публикует odometry
в world, body pose из IMU и linear velocity в world; child_frame_id тоже world.
Bridge ожидает local_frd/body_frd. Прямой remap небезопасен по смыслу данных.
В bridge IMU переводится FRD→FLU: [x,-y,-z] и для gyro, и для accel.
body_T_cam0 должен быть выражен относительно ЭТОЙ FLU IMU, а не сырых FRD осей.
Kalibr cam_T_imu и VINS body_T_cam имеют противоположные направления:
после проверки соглашений необходимо обращение SE(3).

До подключения выхода VINS нужны тесты pose, quaternion, world→body velocity,
ковариаций, timestamp, сброса/скачков estimator, stale odometry и привязки
локального frame к PX4 NED. Нельзя выдавать upstream world за географический ENU.
Не включать synthetic odometry одновременно с реальным источником.
Команды homing оставить enable_homing_commands=false и alignment_valid=false.
Сначала проверить VIO на записанном bag и ручном перемещении стенда с отключённой
передачей управления, затем повторить информационный тест PX4.

## Что можно завершить после подключения

1. Изображение/режим/частота и запись camera+IMU.
2. Intrinsics; затем camera–IMU extrinsics/time offset и проверка отчётов.
3. Настройка установленного VINS, проверка на bag и преобразование odometry.
4. Реальный Wi-Fi heartbeat, отмена RETURN/WAIT_LINK.
5. Повтор полного PX4 bench на новой версии.

Лётные требования 500 m/10 m и реальное выполнение возврата/посадки без БВС
подтвердить нельзя. Этот документ не означает готовность к полёту.

Источники соглашений: [VINS config](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/config/euroc/euroc_mono_imu_config.yaml),
[VINS odometry](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp).
Установленный форк ещё требуется сверить.

[Kalibr: совместная калибровка камеры и IMU](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration).
