# mvch — подготовка к ЧВТ-2026

Репозиторий для самостоятельного изучения и тренировок по модулям ЧВТ.

- `module2/module_b.py` — модуль **Б**, навигация РМК-2 по ArUco.
- `module3/` — модуль **В**, датасет и распознавание инструментов.
- `module4/` — модуль **Г**, веб-FMS для двух роверов на React/Bun.
- `module5/` — модуль **Д**, единый сценарий RMC1 + RMC2.

Все актуальные модули находятся в `main`. Ветки `module2`–`module5` сохранены
как промежуточные этапы разработки.

## Запуск в Ubuntu VM

Нужен установленный официальный симулятор, ROS 2 Jazzy, `rclpy`, `tf2_ros`.
Дополнительные pip-пакеты и сборка colcon самого решения не нужны.

### Если ROS2 или симулятор отсутствуют

Штатная площадка должна предоставлять готовые ROS2 Jazzy, Webots и workspace
симулятора. Сначала попросите эксперта восстановить окружение. Для самостоятельной
установки нужен Ubuntu 24.04; ROS2 устанавливается по
[официальной инструкции](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
После подключения ROS-репозитория:

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-moveit \
  ros-jazzy-tf2-tools ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-webots-ros2 python3-colcon-common-extensions python3-rosdep ros-dev-tools
```

Webots ставится из [репозитория Cyberbotics](https://cyberbotics.com/doc/guide/installation-procedure):

```bash
sudo install -d /etc/apt/keyrings
sudo wget -q -O /etc/apt/keyrings/Cyberbotics.asc https://cyberbotics.com/Cyberbotics.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" \
  | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update
sudo apt install -y webots
```

Исходники официального симулятора:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://gitlab.mobird.dev/fms_group/ar_webots_fms_ros2.git
git clone https://gitlab.mobird.dev/fms_group/ar_arm95_moveit_config.git
git clone https://gitlab.mobird.dev/fms_group/ar_aruco_detect_ros2.git
git clone https://gitlab.mobird.dev/fms_group/ar_dual_lidar_merge_ros2.git
git clone https://gitlab.mobird.dev/fms_group/ar_nav_ros2.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -y --ignore-src --rosdistro jazzy
colcon build --symlink-install
```

Терминал 1 — штатный симулятор (один экземпляр):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ar_webots_fms_ros2 module2.launch.py
```

Терминал 2 — решение:

```bash
cd ~/mvch
git pull --ff-only
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 module2/module_b.py --target 14
```

1. Поставьте РМК-2 центром над любой меткой. Код сам определит стартовый ID и
   начальную ориентацию.
2. Код заранее выведет кратчайшие маршруты к цели и обратно. **Enter** запускает
   движение к цели.
3. После `TARGET_REACHED` поставьте динамическое препятствие и нажмите **Enter**.
4. Код ещё раз выведет обратный маршрут. Следующий **Enter** запускает возврат.
5. Если лидар увидит преграду на ребре, ровер вернётся к предыдущей метке,
   исключит ребро и выведет `ROUTE_REPLANNED`.
6. Успешный конец — `MISSION_FINISHED`. `Ctrl+C` публикует нулевую скорость.

На физическом РМК-2 команда такая же:

```bash
python3 module2/module_b.py --target 14
```

Единственный аргумент — целевой ID:

```bash
python3 module2/module_b.py --target 14
```

Остальные настройки находятся в начале `module2/module_b.py`:

- `ROWS`, `COLUMNS` — площадка 5×5; для старого симулятора поставить 6 и 6;
- `SPACING` — шаг между метками;
- `BLOCKED` — закрытые ID, например `{7, 8, 13}`;
- `MAX_SPEED`, `MAX_ANGULAR` — скорости;
- `STOP_DISTANCE`, `TOLERANCE`, `SCAN_TOPIC` — lidar и точность остановки.

## Как устроен код

Читайте в таком порядке: `shortest_path` → `localize` → `drive_to` → `follow` →
`main`. BFS строит кратчайший путь только по соседним свободным ячейкам.

Схема поля: `0=(0,0)`, `1=(0,+1)`, `5=(-1,0)`. Старт может быть на любой
метке. По TF видимой ArUco код связывает сетку с odometry, поэтому начальная
ориентация тоже может быть любой. Между соседними метками ровер использует
odometry, а на каждой следующей метке снова корректируется по ArUco.

Загруженная карта сама по себе не даёт RMC2 команды «ехать к marker ID». В выданном
workspace RMC2 управляется через `/RMC2/cmd_vel`; Nav2 настроен для RMC1. Поэтому
модуль Б напрямую использует ArUco, odometry и `/RMC2/scan_front`.

## Логи

`module2/reports/module_b_*.jsonl` содержит обнаруженные маркеры, оба маршрута,
препятствия, достигнутые точки и чистое время движения. Паузы на установку
препятствия в это время не входят.

Перед реальным заездом достаточно проверить направление осей, фактический шаг
между метками и дистанцию безопасной остановки. Скрипт не делает отдельный аудит
топиков при старте: он просто стоит, пока не получит odometry, ArUco и её TF.

## Обновление VM с хоста

GitHub: `git@github.com:mavxa/mvch.git` (приватный).
Для текущей VM используется локальное зеркало хостового Git без передачи ключей GitHub.
На хосте запущен read-only Git daemon только для этого репозитория:

```bash
git daemon --reuseaddr --export-all --listen=127.0.0.1 --port=9418 \
  --base-path=/home/mavxa/zed/chvtFinal /home/mavxa/zed/chvtFinal/mvch
```

В VM с QEMU user networking хост доступен как `10.0.2.2`:

```bash
git clone --branch module2 git://10.0.2.2:9418/mvch ~/mvch
cd ~/mvch
git pull --ff-only
```

SSH VM: `ssh -p 2222 -o HostKeyAlias=chvt-3735b9fd mavxa@127.0.0.1`.
Проброс действует до выключения VM. Восстановить на хосте:

```bash
virsh -c qemu:///system qemu-monitor-command ubuntu24.04-chvt \
  --hmp 'hostfwd_add hostnet0 tcp:127.0.0.1:2222-:22'
```

Пароли и ключи в репозитории не хранятся.
