# QGC через точку доступа VIO-HOMING

Обновлено 17.10.2026. Relay перенесён на Jetson: владелец подтвердил QGC через
Ethernet, затем работу по Wi-Fi AP с просмотром камеры. Реальная PX4 console
показала GCS255/190 и companion245/191 на TELEM2. Текущая скорость460800,
MAV_1_RATE30000. Единый запуск теперь через vio-wireless.service;
пошаговый первичный запуск ниже — для обслуживания при остановленном сервисе.
См. [актуальную инструкцию](WIRELESS_SETUP.md).

## Тракт

QGC на Mac ↔ UDP по wlan0 Jetson (192.168.50.1) ↔ существующий bridge ↔
/dev/ttyTHS1 ↔ PX4. Оригинальные MAVLink packets пересылаются без перепаковки:
system/component ID, sequence и подпись не заменяются companion ID.
В обратную сторону пересылаются принятые UART packets, включая HEARTBEAT,
PARAM_VALUE, COMMAND_ACK, STATUSTEXT и телеметрию.

Настройки по умолчанию: Jetson bind 192.168.50.1:14555,
QGC destination <адрес Mac по Wi-Fi>:14550. Только один фиксированный peer IP,
GCS system=255 component=190. При другом ID QGC его надо явно сверить и задать
в private ROS parameters bridge. IP и ID — фильтры, НЕ криптографическая
аутентификация. Пароль AP не хранится в репозитории. Не открывать UDP наружу.

QGC должен присылать HEARTBEAT типа MAV_TYPE_GCS (6) с ожидаемыми ID.
После первого heartbeat пакеты этого GCS передаются PX4, пока heartbeat свежее
4 секунд. Telemetry из PX4 передаётся также до первого GCS heartbeat, чтобы QGC
мог обнаружить аппарат. Нет очереди повторной отправки команд; reconnect UART
сбрасывает heartbeat и UDP receive queue. Трафик чтения ограничен 32 datagrams
за один цикл. Поток высокого rate и загрузку UART нужно проверить на стенде.

## Запуск после переноса и сборки

1. Подключить Mac к AP, выяснить IP Mac в 192.168.50.0/24. Для устойчивого
адреса настроить DHCP reservation отдельно; автоматического обучения peer нет.
2. Сохранить Ethernet SSH. Остановить штатный автозапуск companion его реальным
service manager (имя сервиса сначала проверить). Не запускать второй UART bridge.
3. Запустить замену (пример адреса Mac, заменить фактическим):

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch vio_homing qgc_wifi.launch qgc_peer_ip:=192.168.50.2
```

4. В QGC создать UDP link с local/listening port 14550 и target
192.168.50.1:14555. Убедиться, что порт 14550 не занят другим экземпляром QGC.
На время доказательной Wi-Fi проверки отключить USB MAVLink link в QGC,
иначе он может получать аппарат через USB. Сначала только telemetry и
безопасный запрос параметра/версии. Не нажимать ARM/Takeoff для проверки канала.
5. Сверить реальные QGC sysid/compid и частоту heartbeat. Если они отличаются,
адаптировать параметры, не снижать проверки PX4.

## Потеря управления

В qgc_wifi.launch monitor получает события `/vio_homing/qgc_heartbeat`
(std_msgs/Empty) только от валидного GCS heartbeat из Wi-Fi UDP входа bridge.
Ни USB heartbeat QGC, ни heartbeat Pixhawk, ни доступность AP не заменяют его.
Существующие ROS link_state/link_age/link_alive сохраняют типы и имена.
При обычном 1 Hz heartbeat: LINK_OK age<2 s; LINK_DEGRADED 2..4 s;
LINK_LOST >=4 s. До первого heartbeat WAITING: сам запуск без оператора не
должен инициировать возврат. Проверять действительную частоту QGC на стенде.
Старый UDP prefix heartbeat на15050 сохраняется в обычном companion.launch.

Проверка: сначала подтверждённый LINK_OK; затем выключить Wi-Fi Mac или закрыть
QGC → DEGRADED → LOST; восстановить QGC → OK. Ethernet оставить. Это обнаруживает
потерю heartbeat, но не доказывает двустороннее качество управления: при
одностороннем отказе downlink heartbeat uplink может продолжать приходить.
Для будущего полётного варианта потребуется отдельная проверка двустороннего
канала/подтверждения статуса, а не только одного heartbeat.

## Управление и ограничения

Relay способен переносить сообщения ARM, Takeoff, mode, MANUAL_CONTROL и другие
сообщения QGC; сам не генерирует эти команды и не принимает решения о возможности
выполнения. ACK/отказ возвращается из PX4. Без принятого запроса, валидной навигации
и подходящего состояния управления физического выполнения не будет.
Джойстик QGC и движения по карте имеют разные требования к режимам/навигации;
их настройку следует выполнять отдельно по фактической версии PX4.

Текущий homing adapter разрешает setpoint/LAND только DISARMED: это предыдущий
стендовый режим. **Возврат в полёте этим изменением не реализован.** qgc_wifi.launch
оставляет enable_homing_commands=false, alignment_valid=false и камеру выключенной.
Это канал QGC и детектор потери GCS, а не готовое управление БВС.
Для полётного RETURN потребуются: реальная VIO, валидная привязка к NED,
управление OFFBOARD/mode handover, приоритеты GCS/homing, взаимодействие с PX4
GCS/manual/offboard failsafe и сценарии потери/восстановления. Параметры PX4
этим кодом не меняются. На текущем стенде ARM/взлёт не выполнялись.

## Необходимая аппаратная проверка

- QGC обнаружил аппарат именно по Wi-Fi; параметры/версия пришли от PX4.
- Реальные sysid/compid, частота heartbeat, отсутствие ложного DEGRADED.
- Безопасная команда запроса → UART → PX4 → ACK/ответ → QGC.
- Потеря и восстановление Wi-Fi, закрытие QGC, перезапуск bridge/UART.
- IMU ~100Hz, ODOMETRY ~30Hz и TIMESYNC не ухудшились; UART загрузка/latency.
- Полный synthetic homing отдельным тестом, без ARM и с явным стендовым enable.

Источники: [MAVLink heartbeat](https://mavlink.io/en/services/heartbeat.html),
[routing](https://mavlink.io/en/guide/routing.html),
[QGC joystick](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html).
