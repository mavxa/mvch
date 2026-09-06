# Модуль Г — веб-интерфейс FMS

Одностраничная панель управления двумя роверами. Стек: React, TypeScript, Vite,
Tailwind CSS 4, Bun и небольшой ROS 2-мост на Python.

## Что реализовано

- выбор RMC1 или RMC2;
- положение, скорость и история движения обоих роверов;
- ArUco-граф 6 × 6 с координатами маркеров 0–35;
- карта занятости RMC1 из `/map`;
- облако точек `/RMC*/scan`, повернутое и перенесённое вместе с ровером;
- план Nav2 из `/RMC*/plan` и текущая целевая точка;
- ручное управление кнопками и клавишами W/A/S/D, для RMC1 также Q/E;
- watchdog: после отпускания клавиши или потери окна скорость обнуляется;
- установка текущей позиции через `/RMC*/initialpose` без движения ровера;
- автономная цель через Nav2, с простым контроллером по одометрии как fallback;
- независимый аварийный стоп с отменой активной Nav2-задачи;
- схват RMC1 и лифт RMC2;
- вольтаж из стандартных ROS-топиков, если такой топик доступен в окружении;
- WebSocket-телеметрия с частотой 5 Гц;
- mock-режим для проверки вебки без ROS/Webots.

## Запуск на VM

Нужен Bun и уже собранное ROS 2-окружение соревнования.

```bash
cd ~/scripts/mvch/module4
bun install
bun run build
```

В каждом ROS-терминале сначала подключить Jazzy и workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

1. Запустить карту с двумя роверами:

```bash
ros2 launch ar_webots_fms_ros2 module5.launch.py
```

2. Для полноценной автономной навигации RMC1 запустить Nav2:

```bash
ros2 launch ar_nav_ros2 bringup_launch.py namespace:=/RMC1 robot_type:=rmc1
```

Если Nav2 в образе не установлен или action недоступен, ROS-мост автоматически
использует простой контроллер по одометрии. Он доводит оба ровера до x/y/yaw, но
не строит обход сложных препятствий, поэтому Nav2 для RMC1 предпочтительнее.

3. В третьем, также ROS-sourced терминале запустить FMS:

```bash
cd ~/scripts/mvch/module4
bun run start
```

Открыть <http://localhost:3000>. Сервер Bun сам запускает
`server/ros_bridge.py`; отдельно стартовать Python-файл не нужно.

Для разработки с Vite HMR:

```bash
bun run dev
```

Страница будет на <http://localhost:5173>, API — на порту 3001.

## Быстрая проверка без симулятора

```bash
bun run build
MOCK_ROS=1 bun run start
```

Mock не используется на соревновании. Он нужен только для проверки кнопок,
WebSocket и компоновки интерфейса.

## ROS-топики

| Назначение | RMC1 | RMC2 |
| --- | --- | --- |
| Одометрия | `/RMC1/odometry` | `/RMC2/odometry` |
| Лидар | `/RMC1/scan` | `/RMC2/scan` |
| Ручная скорость | `/RMC1/cmd_vel` | `/RMC2/cmd_vel` |
| Начальная позиция | `/RMC1/initialpose` | `/RMC2/initialpose` |
| Цель | `/RMC1/goal_pose` | `/RMC2/goal_pose` |
| План | `/RMC1/plan` | `/RMC2/plan` |
| Рабочий инструмент | `.../gripper_trajectory_controller/joint_trajectory` | `/RMC2/lift` |

Дополнительно используются `/map`, `/RMC2/aruco_id` и
`/RMC2/lift_status`.

Симулятор из открытого пакета не публикует напряжение аккумуляторов. Мост уже
подписан на `/RMC*/battery_state` (`sensor_msgs/BatteryState`) и
`/RMC*/battery_voltage` (`std_msgs/Float64`). Если на площадке топика тоже не
будет, интерфейс честно покажет `нет топика`, а не фиктивное значение.

## HTTP API

- `GET /api/health` — состояние Bun и ROS-моста;
- `GET /api/state` — последний снимок телеметрии;
- `POST /api/command` — команда управления;
- `WS /ws` — телеметрия в реальном времени.

Пример остановки RMC1:

```bash
curl -X POST http://localhost:3000/api/command \
  -H 'content-type: application/json' \
  -d '{"type":"emergency","robot":"RMC1","active":true}'
```

## Проверки

```bash
bun run check
bun test
bun run build
python3 -m py_compile server/ros_bridge.py
```
