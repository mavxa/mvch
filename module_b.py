#!/usr/bin/env python3
"""Модуль Б: один запуск, маршрут туда, пауза эксперта, маршрут обратно.

python3 module_b.py --target 14 --sim
Команды в терминале: Enter / go, stop, resume, return, quit.
"""

import argparse
import heapq
import json
import math
import queue
import re
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


# Все параметры модуля Б находятся здесь. Перед реальным полем меняются в одном месте.
ROWS = 6
COLS = 6
MARKER_SPACING = 1.0
BLOCKED_MARKERS = set()

ROBOT_NAMESPACE = "RMC2"
MARKER_PREFIX = "aruco_"
# RMC2.proto: край корпуса находится примерно в 0.373 м от base_link.
# Четыре сантиметра запаса оставляют проходимым узкий коридор у marker 33.
ROBOT_HALF_LENGTH = 0.375
ROBOT_HALF_WIDTH = 0.375
SAFETY_MARGIN = 0.04
ROUTE_CLEARANCE = 0.44

MAX_LINEAR = 0.25
MAX_ANGULAR = 0.45
LINEAR_ACCEL = 0.35
ANGULAR_ACCEL = 0.80
POSITION_TOLERANCE = 0.025
YAW_TOLERANCE = 0.045
MOVE_YAW_TOLERANCE = 0.14
SENSOR_TIMEOUT = 1.5
MARKER_TIMEOUT = 0.8
WAYPOINT_TIMEOUT = 50.0
MISSION_TIMEOUT = 330.0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def compose(a, b):
    """Переносит позу b из системы a в её родительскую систему."""
    x, y, yaw = a
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c*b[0] - s*b[1], y + s*b[0] + c*b[1], wrap(yaw+b[2])


def inverse(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return -c*x-s*y, s*x-c*y, -yaw


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def segment_distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx+dy*dy
    if length2 == 0:
        return distance(point, a)
    t = clamp(((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2, 0, 1)
    return distance(point, (a[0]+t*dx, a[1]+t*dy))


class Field:
    """Регулярная сетка ArUco и кратчайший путь по ней."""

    def __init__(self):
        self.poses = {}
        self.graph = {}
        for row in range(ROWS):
            for col in range(COLS):
                marker = row*COLS+col
                self.poses[marker] = (-row*MARKER_SPACING, col*MARKER_SPACING, 0.0)
                self.graph[marker] = [
                    r*COLS+c for r, c in
                    ((row+1, col), (row, col+1), (row-1, col), (row, col-1))
                    if 0 <= r < ROWS and 0 <= c < COLS
                ]

    def nearest(self, x, y):
        return min(self.poses, key=lambda marker: distance((x, y), self.poses[marker]))

    def route(self, start, goal, obstacle_points=(), forbidden_edges=()):
        """Дейкстра: кратчайший по длине свободный путь."""
        if start not in self.poses or goal not in self.poses:
            raise ValueError("ID старта или цели отсутствует в графе")
        if start in BLOCKED_MARKERS or goal in BLOCKED_MARKERS:
            raise ValueError("Старт или цель заблокированы")

        forbidden = {frozenset(edge) for edge in forbidden_edges}
        costs = {start: 0.0}
        parents = {start: None}
        queue_path = [(0.0, start)]
        while queue_path:
            cost, current = heapq.heappop(queue_path)
            if cost > costs[current]:
                continue
            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = parents[current]
                return path[::-1]
            for neighbor in self.graph[current]:
                edge = frozenset((current, neighbor))
                if neighbor in BLOCKED_MARKERS or edge in forbidden:
                    continue
                a, b = self.poses[current], self.poses[neighbor]
                if any(segment_distance(p, a, b) < ROUTE_CLEARANCE for p in obstacle_points):
                    continue
                new_cost = cost+distance(a, b)
                if new_cost < costs.get(neighbor, math.inf):
                    costs[neighbor] = new_cost
                    parents[neighbor] = current
                    heapq.heappush(queue_path, (new_cost, neighbor))
        raise ValueError(f"Нет безопасного маршрута {start} -> {goal}")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--target", type=int, help="ID целевой метки")
    target.add_argument("--xy", nargs=2, type=float, help="Координаты цели на карте, метры")
    parser.add_argument("--sim", action="store_true", help="Часы Webots /clock")
    parser.add_argument("--target-yaw", type=float, help="Ориентация на цели в системе поля, рад")
    parser.add_argument("--auto", action="store_true", help="Только тренировка: без команд эксперта")
    parser.add_argument("--return-delay", type=float, default=5.0, help="Пауза --auto, секунд симуляции")
    parser.add_argument("--dry-run", action="store_true", help="Показать путь без ROS и без движения")
    parser.add_argument("--start", type=int, default=0, help="Старт только для --dry-run")
    args = parser.parse_args()
    if args.auto and not args.sim:
        parser.error("--auto разрешён только с --sim")
    if args.return_delay < 0 or not math.isfinite(args.return_delay):
        parser.error("--return-delay должен быть конечным неотрицательным числом")
    if args.target_yaw is not None and not math.isfinite(args.target_yaw):
        parser.error("--target-yaw должен быть конечным числом")
    if args.xy and not all(math.isfinite(v) for v in args.xy):
        parser.error("--xy должен содержать конечные числа")
    return args


def main():
    args = arguments()
    field = Field()
    target = args.target if args.target is not None else field.nearest(*args.xy)
    if target not in field.poses or target in BLOCKED_MARKERS:
        raise SystemExit("Целевая метка отсутствует в графе или заблокирована")
    if args.dry_run:
        print("ROUTE", field.route(args.start, target), "TARGET", field.poses[target])
        return

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener, TransformException

    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def tf_pose(transform):
        t = transform.transform
        return t.translation.x, t.translation.y, yaw(t.rotation)

    class Mission(Node):
        def __init__(self):
            super().__init__("mvch_module_b")
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=args.sim)])
            self.cfg = {
                "max_linear": MAX_LINEAR, "max_angular": MAX_ANGULAR,
                "linear_accel": LINEAR_ACCEL, "angular_accel": ANGULAR_ACCEL,
                "position_tolerance": POSITION_TOLERANCE,
                "yaw_tolerance": YAW_TOLERANCE,
                "move_yaw_tolerance": MOVE_YAW_TOLERANCE,
                "sensor_timeout": SENSOR_TIMEOUT, "marker_timeout": MARKER_TIMEOUT,
                "waypoint_timeout": WAYPOINT_TIMEOUT, "mission_timeout": MISSION_TIMEOUT,
            }
            self.robot = {
                "namespace": ROBOT_NAMESPACE, "marker_prefix": MARKER_PREFIX,
                "clearance": ROUTE_CLEARANCE, "half_length": ROBOT_HALF_LENGTH,
                "half_width": ROBOT_HALF_WIDTH, "safety_margin": SAFETY_MARGIN,
            }
            ns = ROBOT_NAMESPACE.strip("/")
            self.base, self.odom_frame = f"{ns}/base_link", f"{ns}/odom"
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 1)
            self.status_pub = self.create_publisher(String, "/mvch/status", 10)
            self.create_subscription(Odometry, f"/{ns}/odometry", self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, f"/{ns}/scan", self.on_scan, qos_profile_sensor_data)
            self.create_subscription(String, f"/{ns}/aruco_id", self.on_marker, 10)
            self.create_subscription(Bool, "/mvch/emergency_stop", self.on_stop, 10)
            self.create_subscription(String, "/mvch/command", lambda m: self.commands.put(m.data), 10)
            self.commands = queue.Queue()
            self.state, self.leg = "WAIT_SENSORS", "outbound"
            self.odom = self.map_from_odom = self.pose = None
            self.odom_wall = self.scan_wall = 0.0
            self.scan = None
            self.points, self.body_points = [], []
            self.lidar_tf_wall = 0.0
            self.lidar_tf_error = "TF лидара ещё не получен"
            self.marker = None
            self.marker_wall = self.marker_stamp = 0.0
            self.marker_distance = math.inf
            self.start = self.current = None
            self.start_yaw = 0.0
            self.goal, self.route, self.index = target, [], 0
            self.forbidden = set()
            self.paused = False
            self.terminal = False
            self.failed = False
            self.v = self.w = 0.0
            self.last_sim = self.now()
            self.last_wall = self.clock_wall = time.monotonic()
            self.last_clock = self.last_sim
            self.leg_started = self.waypoint_started = self.last_sim
            self.total_time = 0.0
            self.last_status = 0.0
            self.wait_reason = ""
            log_dir = Path(__file__).parent / "reports"
            log_dir.mkdir(exist_ok=True)
            self.log_path = log_dir / f"module_b_{datetime.now():%Y%m%d_%H%M%S_%f}.jsonl"
            self.log_file = self.log_path.open("a", encoding="utf-8", buffering=1)
            # Таймер на монотонных часах: остановка сработает даже при паузе /clock.
            self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.log("READY", target=target, config="constants in module_b.py", sim=args.sim,
                     instructions="Enter/go: старт; return: построить возврат; stop/resume; quit")

        def now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def log(self, event, **data):
            item = dict(time=datetime.now().isoformat(timespec="milliseconds"),
                        sim_time=round(self.now(), 3), event=event, leg=self.leg, **data)
            line = json.dumps(item, ensure_ascii=False)
            print(line, flush=True)
            self.log_file.write(line+"\n")

        def on_odom(self, msg):
            p = msg.pose.pose
            pose = p.position.x, p.position.y, yaw(p.orientation)
            if all(math.isfinite(v) for v in pose):
                self.odom, self.odom_wall = pose, time.monotonic()

        def on_scan(self, msg):
            # Пустой/повреждённый скан не считается свободным пространством.
            if not msg.ranges or msg.angle_increment == 0:
                return
            if not any(math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges):
                return
            self.scan, self.scan_wall = msg, time.monotonic()

        def on_marker(self, msg):
            match = re.fullmatch(re.escape(MARKER_PREFIX)+r"(\d+)", msg.data.strip())
            if not match and msg.data.strip().isdigit():
                marker = int(msg.data.strip())
            elif match:
                marker = int(match[1])
            else:
                return
            if marker in field.poses:
                self.marker, self.marker_wall = marker, time.monotonic()

        def on_stop(self, msg):
            if msg.data:
                self.paused = True
                self.stop()
                self.log("EMERGENCY_STOP")
            # false сам по себе не разрешает движение: требуется resume.

        def localize(self):
            if self.odom is None or self.marker is None:
                return
            frame = MARKER_PREFIX+str(self.marker)
            try:
                transform = self.buffer.lookup_transform(self.odom_frame, frame, Time())
                stamp = transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
                if not 0 <= self.now()-stamp < self.cfg["marker_timeout"]:
                    return
                if time.monotonic()-self.marker_wall > self.cfg["sensor_timeout"]:
                    return
                odom_marker = tf_pose(transform)
                self.marker_distance = distance(odom_marker, self.odom)
                if stamp <= self.marker_stamp:
                    return
                correction = compose(field.poses[self.marker], inverse(odom_marker))
                if self.map_from_odom is not None:
                    old_pose = compose(self.map_from_odom, self.odom)
                    new_pose = compose(correction, self.odom)
                    if distance(old_pose, new_pose) > 0.4 or abs(wrap(old_pose[2]-new_pose[2])) > 0.6:
                        return  # Не принимаем скачок распознавания; таймаут остановит у метки.
                self.map_from_odom, self.marker_stamp = correction, stamp
            except TransformException:
                pass

        def scan_points(self):
            scan = self.scan
            try:
                # В этом симуляторе laser_merged повёрнут на 120 градусов!
                mount = tf_pose(self.buffer.lookup_transform(self.base, scan.header.frame_id, Time()))
                # Последний TF достаточно свежий: scan проверяется отдельно по wall-clock.
                # Запрос строго по stamp иногда попадает между двумя odom TF и кратко падает.
                at_scan = tf_pose(self.buffer.lookup_transform(self.odom_frame, self.base, Time()))
            except TransformException as error:
                self.lidar_tf_error = str(error)
                return False
            scan_in_map = compose(compose(self.map_from_odom, at_scan), mount)
            body_from_map = inverse(self.pose)
            points = {}
            for i, r in enumerate(scan.ranges):
                if not math.isfinite(r) or not scan.range_min <= r <= scan.range_max:
                    continue
                angle = scan.angle_min+i*scan.angle_increment
                p = compose(scan_in_map, (r*math.cos(angle), r*math.sin(angle), 0))[:2]
                b = compose(body_from_map, (*p, 0))
                if abs(b[0]) < self.robot["half_length"] and abs(b[1]) < self.robot["half_width"]:
                    continue  # собственный корпус, не внешнее препятствие
                points[(round(p[0]/0.04), round(p[1]/0.04))] = p
            self.points = list(points.values())
            self.body_points = [compose(body_from_map, (*p, 0)) for p in self.points]
            self.lidar_tf_wall = time.monotonic()
            self.lidar_tf_error = ""
            return True

        def plan(self):
            self.stop()
            try:
                self.route = field.route(
                    self.current, self.goal, self.points, self.forbidden
                )
            except ValueError as error:
                self.abort(str(error))
                return
            self.index = 1
            self.state = "WAIT_GO"
            self.log("ROUTE", markers=self.route, goal=self.goal)
            self.log("WAIT_GO", message="Эксперт фиксирует маршрут. Enter/go разрешает движение.")

        def stop(self):
            self.v = self.w = 0.0
            self.pub.publish(Twist())

        def abort(self, reason):
            self.stop()
            self.failed = self.terminal = True
            self.state = "FAILED"
            self.log("FAILED", reason=reason, current=self.current)

        def command(self, command):
            command = command.strip().lower()
            if command in ("stop", "s"):
                self.paused = True
                self.stop()
                self.log("EMERGENCY_STOP")
            elif command == "resume":
                self.paused = False
                self.log("RESUME")
            elif command in ("quit", "q"):
                self.abort("Остановлено оператором")
            elif self.state == "WAIT_RETURN" and command in ("", "return", "go"):
                self.leg, self.goal = "return", self.start
                self.forbidden.clear()
                self.plan()
            elif self.state == "WAIT_GO" and command in ("", "go") and not self.paused:
                # Новая преграда за время паузы: сначала показать новый маршрут.
                try:
                    updated = field.route(
                        self.current, self.goal, self.points, self.forbidden
                    )
                except ValueError as error:
                    self.abort(str(error))
                    return
                if updated != self.route:
                    self.plan()
                    return
                self.state = "DRIVE"
                self.leg_started = self.waypoint_started = self.now()
                self.log("movement_start", markers=self.route)

        def blocked_motion(self, v, w):
            margin = self.robot["safety_margin"]
            # Тормозной путь + время реакции; размеры считаются от центра корпуса.
            braking = v*v/(2*self.cfg["linear_accel"])+abs(v)*0.25
            front = self.robot["half_length"]+margin+braking
            width = self.robot["half_width"]+margin
            if abs(v) > 0.001:
                direction = 1 if v > 0 else -1
                return any(0 < direction*x < front and abs(y) < width for x, y, _ in self.body_points)
            if abs(w) > 0.001:
                return any(math.hypot(x, y) < self.robot["clearance"] for x, y, _ in self.body_points)
            return False

        def send(self, v, w, dt):
            if self.blocked_motion(v, w):
                self.stop()
                return False
            self.v += clamp(v-self.v, -self.cfg["linear_accel"]*dt, self.cfg["linear_accel"]*dt)
            self.w += clamp(w-self.w, -self.cfg["angular_accel"]*dt, self.cfg["angular_accel"]*dt)
            # Поворот на месте и аварийная остановка не должны оставлять линейный хвост.
            if v == 0:
                self.v = 0.0
            if self.blocked_motion(self.v, self.w):
                self.stop()
                return False
            msg = Twist()
            msg.linear.x, msg.angular.z = float(self.v), float(self.w)
            self.pub.publish(msg)
            return True

        def drive(self, dt):
            if self.now()-self.waypoint_started > self.cfg["waypoint_timeout"]:
                self.abort("Таймаут точки: проверить камеру, препятствие и положение робота")
                return
            if self.state == "ALIGN":
                desired = args.target_yaw if self.leg == "outbound" else self.start_yaw
                error = wrap(desired-self.pose[2])
                if abs(error) <= self.cfg["yaw_tolerance"]:
                    self.arrived()
                else:
                    self.send(0, clamp(1.4*error, -self.cfg["max_angular"], self.cfg["max_angular"]), dt)
                return
            if self.index >= len(self.route) and self.state != "BACKTRACK":
                if self.leg == "return" or args.target_yaw is not None:
                    self.state = "ALIGN"
                else:
                    self.arrived()
                return
            marker = self.current if self.state == "BACKTRACK" else self.route[self.index]
            target_pose = field.poses[marker]
            dx, dy = target_pose[0]-self.pose[0], target_pose[1]-self.pose[1]
            d = math.hypot(dx, dy)
            fresh = self.marker == marker and 0 <= self.now()-self.marker_stamp < self.cfg["marker_timeout"]
            if d < self.cfg["position_tolerance"] and fresh and self.marker_distance < 0.04:
                self.stop()
                if self.state == "BACKTRACK":
                    self.log("BACKTRACK_DONE", marker=marker)
                    try:
                        self.route = field.route(
                            self.current, self.goal, self.points, self.forbidden
                        )
                    except ValueError as error:
                        self.abort(str(error))
                        return
                    self.index, self.state = 1, "DRIVE"
                    self.log("ROUTE_REPLANNED", markers=self.route)
                else:
                    self.current = marker
                    self.index += 1
                    self.log("WAYPOINT", marker=marker, pose=list(self.pose), marker_error=self.marker_distance)
                self.waypoint_started = self.now()
                return
            if d < self.cfg["position_tolerance"] and not fresh:
                self.stop()  # Без свежей метки достижение НЕ засчитываем.
                return
            error = wrap(math.atan2(dy, dx)-self.pose[2])
            # Если следующая метка сзади, едем к ней задним ходом. Это не даёт
            # роботу разворачиваться корпусом вплотную к стеллажам у marker 33.
            reverse = self.state == "BACKTRACK" or abs(error) > math.pi/2
            if reverse:
                error = wrap(error-math.pi)
            v = min(self.cfg["max_linear"], 0.8*d)
            if reverse:
                v = -min(v, 0.12)
            if abs(error) > self.cfg["move_yaw_tolerance"]:
                v = 0
            w = clamp(1.5*error, -self.cfg["max_angular"], self.cfg["max_angular"])
            if not self.send(v, w, dt) and self.state == "DRIVE":
                edge = (self.current, marker)
                self.forbidden.add(edge)
                self.log("OBSTACLE", edge=edge, pose=list(self.pose))
                # Вернуться по уже пройденному ребру, затем ехать по новому графу.
                self.state = "BACKTRACK"
                self.waypoint_started = self.now()
                self.log("BACKTRACK", marker=self.current)

        def arrived(self):
            self.stop()
            elapsed = self.now()-self.leg_started
            self.total_time += elapsed
            self.log("movement_stop", marker=self.goal, seconds=round(elapsed, 3))
            if self.leg == "outbound":
                self.state = "WAIT_RETURN"
                self.return_wait = self.now()
                self.log("TARGET_REACHED", marker=self.goal,
                         message="Выставьте препятствие. return/Enter строит обратный маршрут.")
            else:
                self.state, self.terminal = "FINISHED", True
                self.log("MISSION_FINISHED", marker=self.start, movement_seconds=round(self.total_time, 3))

        def tick(self):
            wall, now = time.monotonic(), self.now()
            dt = clamp(now-self.last_sim, 0, 0.15)
            if now < self.last_sim-0.01:
                self.abort("Часы симуляции сброшены; запустите новую попытку")
                return
            if now != self.last_clock:
                self.clock_wall, self.last_clock = wall, now
            self.last_sim = now
            self.localize()
            if self.map_from_odom is not None and self.odom is not None:
                self.pose = compose(self.map_from_odom, self.odom)
            reasons = []
            if self.pose is None:
                reasons.append("нет локализации")
            if self.scan is None:
                reasons.append("нет scan")
            if wall-self.odom_wall >= SENSOR_TIMEOUT:
                reasons.append(f"odometry старше {wall-self.odom_wall:.2f}с")
            if wall-self.scan_wall >= SENSOR_TIMEOUT:
                reasons.append(f"scan старше {wall-self.scan_wall:.2f}с")
            if wall-self.clock_wall >= SENSOR_TIMEOUT:
                reasons.append(f"clock старше {wall-self.clock_wall:.2f}с")
            healthy = not reasons
            if healthy and not self.scan_points():
                # Один краткий промах TF не должен ставить миссию на вечную паузу.
                if wall-self.lidar_tf_wall >= SENSOR_TIMEOUT:
                    reasons.append("TF лидара: "+self.lidar_tf_error)
                    healthy = False
            if not healthy:
                self.stop()
                if self.state != "WAIT_SENSORS" and not self.paused:
                    self.paused = True
                    self.log("SENSOR_STOP", reason="; ".join(reasons),
                             message="После восстановления введите resume")
            while not self.commands.empty():
                command = self.commands.get_nowait()
                if healthy or command.strip() in ("stop", "quit", "q", "s"):
                    self.command(command)
            if not healthy or self.paused or self.terminal:
                self.stop()
                return
            if wall-self.last_status > 1:
                msg = String()
                msg.data = json.dumps(dict(state=self.state, leg=self.leg, pose=self.pose,
                                           current=self.current, route=self.route))
                self.status_pub.publish(msg)
                self.last_status = wall
            if self.state == "WAIT_SENSORS":
                if self.marker_distance > 0.04:
                    return
                self.start = self.current = self.marker
                self.start_yaw = self.pose[2]
                self.log("LOCALIZED", marker=self.start, pose=list(self.pose))
                self.plan()
            elif self.state == "WAIT_GO":
                self.stop()
                if args.auto:
                    self.command("go")
            elif self.state == "WAIT_RETURN":
                self.stop()
                if args.auto and now-self.return_wait >= args.return_delay:
                    self.command("return")
            else:
                if self.total_time+now-self.leg_started > self.cfg["mission_timeout"]:
                    self.abort("Превышено время миссии")
                else:
                    self.drive(dt)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Mission()
    quitting = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: quitting.set())
    signal.signal(signal.SIGTERM, lambda *_: quitting.set())

    def read_commands():
        for line in sys.stdin:
            node.commands.put(line)

    threading.Thread(target=read_commands, daemon=True).start()
    try:
        while rclpy.ok() and not quitting.is_set() and not node.terminal:
            rclpy.spin_once(node, timeout_sec=0.05)
        if quitting.is_set() and not node.terminal:
            node.abort("SIGINT/SIGTERM")
    except Exception as error:
        node.abort(f"{type(error).__name__}: {error}")
        raise
    finally:
        # Перед закрытием соединения несколько раз отправить настоящий нулевой Twist.
        for _ in range(5):
            node.stop()
            time.sleep(0.04)
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()
    if node.failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
