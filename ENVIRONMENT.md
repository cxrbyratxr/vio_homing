# Окружение

Актуализация 17.10.2026: Jetson Nano, ROS Melodic/Python2.7, PX4 1.16.0.
Репозиторий: `/home/jetson/catkin_ws/src/vio_homing`; Kalibr: `~/kalibr_ws`.
UART460800, AP192.168.50.1, Mac192.168.50.24, камера640×480 YUYV/bgr8 30Гц,
IMU около200Гц. Запуск через vio-wireless.service. Подробности в
[README](README.md) и [статусе](PROJECT_STATUS.md).
Остальной текст ниже — историческая запись установки; версии зависимостей
и старые пути перед повторной установкой следует сверять с фактической системой.

### Док описывает настройки и окружение используемой для разработки Jetson Nano 4GB

### Общее
**ОС:** [JetPack 4.6.2 (Ubuntu 18)](https://developer.nvidia.com/embedded/jetpack-sdk-462)

## ROS
Установлен **ROS Melodic**, пакет `ros-melodic-desktop-full`. Источник: [JetsonHacksNano/isntallROS](https://github.com/JetsonHacksNano/installROS)

Запущены оба скрипта:
- installROS.sh (`--package ros-melodic-desktop-full`)
- setupCatkinWorkspace.sh

## Рабочее пространство 
Рабочее пространство Catkin (**Catkin Workspace**) находится по пути:
`/home/jetson/catkin_ws`

Данный репозиторий ([TheMrKan/vio-homing](https://github.com/TheMrKan/vio-homing)) склонирован сюда: `catkin_ws/src/vio-homing`.
В нем предполагается держать весь наш исходный код, конфиги и т. д. 

В `catkin_ws/src/VINS-Fusion` находится форк оригинального проекта от Андрея. Основное отличие от оригинала - убрана буферизация в некоторых топиках.


## Другое
- **VINS Fusion** устанавливался по [официальной доке](https://github.com/hkust-aerial-robotics/vins-fusion).
- Для него необходим **Ceres Solver**. Устанавливался по [официальной доке](http://ceres-solver.org/installation.html#linux). **Ceres Solver v1.12.0** собран, лежит в `/home/jetson/ceres-bin/`
- Для сборки Ceres Solver пришлось **обновить Cmake** (стандартный 3.10.2 -> 3.22). Системный Cmake не тронут, обновленный 3.22 установлен в `/usr/local` через бинарник с cmake.org.
- **ВАЖНО!** Установка с **main** ветки репозитория Ceres Solver не работает, обязательно `git checkout 1.22.0` перед сборкой (версия **1.22.0** взята из [официального Dockerfile](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/docker/Dockerfile#L3)).
- Перед запуском `catkin_make` нужно установить через `apt-get install` следующие пакеты (взяты из [официального Dockerfile](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/docker/Dockerfile#L16)):
- - `ros-melodic-cv-bridge`
- - `ros-melodic-image-transport`
- - `ros-melodic-message-filters`
- - `ros-melodic-tf`
- - `ros-melodic-perception`
- 



