# Базовые примеры для реального ROS 2

Это короткие учебные скрипты, которые работают через штатные ROS 2 интерфейсы
роверов: `cmd_vel`, `odometry`, `scan_front`, `aruco_id`, TF и `lift`. Имена
сверены с выданной документацией физических РМК от 08.09.2026. В них нет
прямого управления моторами и нет подмены симулятора. Один и тот же ROS API можно
проверять в Webots, а затем перенести на физический РМК после сверки имён топиков,
TF, размеров поля и безопасных скоростей.

Это не готовые конкурсные решения и не замена аппаратному аварийному STOP.

Перед запуском:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Сначала проверьте реальные интерфейсы:

```bash
ros2 topic list | sort
ros2 topic info /RMC2/cmd_vel -v
ros2 topic info /RMC2/odometry -v
ros2 topic info /RMC2/scan_front -v
ros2 topic info /RMC2/camera_bottom/aruco_id -v
ros2 topic echo /RMC2/camera_bottom/aruco_id --once
ros2 run tf2_ros tf2_echo RMC2/odom aruco_0
```

Практические примеры:

```bash
# Проехать RMC2 от текущей метки до ArUco 14 на физическом ROS
python3 basic/drive_to_aruco.py 14

# То же в Webots; только здесь нужен --sim
python3 basic/drive_to_aruco.py 14 --sim

# Равнозначная форма с именованным аргументом
python3 basic/drive_to_aruco.py --sim --target 14

# Закрытые клетки и уменьшенная реальная скорость
python3 basic/drive_to_aruco.py 14 --blocked 8 9 --max-speed 0.08

# Короткая проверка движения; lidar обязателен, после 1 с будет stop
python3 basic/move_rmc.py --robot RMC1 --x 0.15 --seconds 1

# Поворот RMC2 на месте
python3 basic/move_rmc.py --robot RMC2 --yaw 0.3 --seconds 1

# Статус положения и лидара
python3 basic/watch_robot.py --robot RMC1

# Управление лифтом RMC2
python3 basic/lift_rmc2.py up
python3 basic/lift_rmc2.py down

# Остановить оба ровера
python3 basic/stop_all.py
```

`drive_to_aruco.py` по умолчанию использует реальную сетку 5×5: `0=(0,0)`,
`1=(0,+1 м)`, `5=(-1 м,0)`. С `--sim` выбирается сетка Webots 6×6, где
`6=(-1 м,0)`. Ровер должен стартовать центром над видимой меткой.
Скрипт сам определяет старт, строит BFS-маршрут, печатает его до движения, затем
едет по odometry и уточняет положение через TF каждой ArUco. Без свежих odometry,
лидара, метки на финише или при препятствии ближе `--stop-distance` он отправляет
нулевой `Twist` и завершает работу.

На реальном поле сначала измерьте шаг сетки и при необходимости задайте
`--spacing`. Если организаторы используют другой топик лидара, передайте
`--scan-topic /нужный/топик`. Не используйте `--yes` в первом физическом заезде.

По выданной документации нижняя камера и `aruco_id` есть у RMC2. Поэтому реальный
запуск `drive_to_aruco.py` разрешён только для RMC2. Подъём лифта публикует `0.05`,
опускание — `0.0`, а статус читается с обязательным QoS `Transient Local`.

У RMC1 доступно боковое движение (`linear.y`), у RMC2 оно принудительно запрещено
в `move_rmc.py`. Режим `--no-lidar` предназначен только для вывешенного стенда,
где колёса не могут переместить робот.

`grid_route.py` оставлен как офлайн-проверка графа. Он ничего не публикует в ROS.
Для реального поля используется его дефолт `--size 5`, для Webots добавьте
`--size 6`.
