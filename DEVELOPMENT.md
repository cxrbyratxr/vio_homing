# Стиль и обслуживание кода

Runtime проекта: ROS Melodic, Python 2.7. Скрипты и библиотеки сохраняют совместимость с этим окружением.

## Форматирование

Python: PEP 8, 4 пробела, строки до79 символов, LF. Настройки YAPF/pycodestyle находятся в setup.cfg, настройки редакторов — в .editorconfig. Для ROS launch/XML используется отступ2 пробела. Архивные verification-логи сохраняются как были получены.

Инструменты форматирования запускаются на Mac в отдельном Python3 окружении:

```bash
python3 -m venv /tmp/vio-format-env
/tmp/vio-format-env/bin/python -m pip install -r requirements-format.txt
/tmp/vio-format-env/bin/yapf --style setup.cfg --in-place --recursive scripts src test setup.py
/tmp/vio-format-env/bin/pycodestyle scripts src test setup.py
```

YAPF и pycodestyle не являются runtime-зависимостями Jetson. E402 допускается для выбора MAVLINK20 до импорта pymavlink; W503/W504 допускают поддерживаемые PEP8 варианты переноса операторов. Остальные правила pycodestyle включены.

## Ограниченные оптимизации

- FrdToNed один раз вычисляет cos/sin при создании фиксированной привязки, вместо повторения для каждой цели.
- TrajectoryRecorder создаёт одну копию нужного участка пути и разворачивает её на месте: исключён второй список ссылок на PoseStamped.
- mavlink_tx_event — transient, non-latched диагностика. Без подписчиков JSON и ROS String не создаются; с подписчиками прежняя структура сообщений сохранена. Latched transport status по-прежнему публикуется всегда.

Топики, MAVLink type mask, оси/знаки, timeout, блокировки, условия отмены и однократность LAND сохраняются. Измерений ускорения в этом этапе нет.

## Сборка

```bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make -j2
```

По просьбе владельца проекта после этой правки функциональные, ROS и PX4 bench тесты не запускались. Предыдущие логи относятся к указанным в них версиям проекта.
