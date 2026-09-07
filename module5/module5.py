#!/usr/bin/env python3
"""Модуль Д: один скрипт для складского кейса RMC1 + RMC2."""

import argparse
import json
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tempfile import NamedTemporaryFile
from types import SimpleNamespace


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
ROWS = 6
COLS = 6
MARKER_SPACING = 1.0
ROBOT_NAMES = ("RMC1", "RMC2")


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def marker_xy(marker_id):
    """Координаты поля: 0=(0,0), 6=(-1,0), 1=(0,1)."""
    row, column = divmod(marker_id, COLS)
    return -row * MARKER_SPACING, column * MARKER_SPACING


def neighbours(marker_id):
    row, column = divmod(marker_id, COLS)
    result = []
    for dr, dc in ((-1, 0), (0, -1), (0, 1), (1, 0)):
        nr, nc = row + dr, column + dc
        if 0 <= nr < ROWS and 0 <= nc < COLS:
            result.append(nr * COLS + nc)
    return result


def shortest_path(start, goal, blocked=()):
    """BFS по регулярному графу ArUco."""
    forbidden = set(blocked) - {start, goal}
    queue = deque([start])
    previous = {start: None}
    while queue:
        current = queue.popleft()
        if current == goal:
            break
        for candidate in neighbours(current):
            if candidate not in forbidden and candidate not in previous:
                previous[candidate] = current
                queue.append(candidate)
    if goal not in previous:
        raise ValueError(f"Нет маршрута {start} -> {goal}; занято: {sorted(forbidden)}")
    route = []
    current = goal
    while current is not None:
        route.append(current)
        current = previous[current]
    return list(reversed(route))


def face_marker(position, target):
    x1, y1 = marker_xy(position)
    x2, y2 = marker_xy(target)
    return math.atan2(y2 - y1, x2 - x1)


def parse_markers(value):
    if not value.strip():
        return set()
    try:
        return {int(item.strip()) for item in value.split(",")}
    except ValueError as error:
        raise argparse.ArgumentTypeError("Нужны ID через запятую, например 31,35") from error


class EventLog:
    def __init__(self):
        directory = HERE / "logs"
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f"module5_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
        self.lock = threading.Lock()

    def write(self, event, **fields):
        record = {"time": datetime.now().isoformat(timespec="milliseconds"), "event": event}
        record.update(fields)
        line = json.dumps(record, ensure_ascii=False)
        with self.lock:
            print(line, flush=True)
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")


@dataclass
class Scenario:
    rmc1_start: int
    rmc2_start: int
    rack: int
    assembly: int
    assembly_approach: int
    delivery: int
    delivery_approach: int
    blocked: set

    def validate(self):
        points = {
            "rmc1-start": self.rmc1_start,
            "rmc2-start": self.rmc2_start,
            "rack": self.rack,
            "assembly": self.assembly,
            "assembly-approach": self.assembly_approach,
            "delivery": self.delivery,
            "delivery-approach": self.delivery_approach,
        }
        for name, marker in points.items():
            if marker not in range(ROWS * COLS):
                raise ValueError(f"{name}: ID {marker} вне диапазона 0..35")
        if self.assembly_approach not in neighbours(self.assembly):
            raise ValueError("assembly-approach должен быть соседней ячейкой")
        if self.delivery_approach not in neighbours(self.delivery):
            raise ValueError("delivery-approach должен быть соседней ячейкой")
        if self.assembly in self.blocked or self.rack in self.blocked:
            raise ValueError("Активный стеллаж и точка комплектации не должны быть в --blocked")

    def routes(self):
        # Полка сдачи и чужие стеллажи считаются препятствиями для RMC2.
        rmc2_blocked = self.blocked | {self.delivery}
        # Для RMC1 стеллаж уже стоит в комплектации, а полка занята физически.
        rmc1_blocked = self.blocked | {self.assembly, self.delivery}
        return {
            "rmc2_to_rack": shortest_path(self.rmc2_start, self.rack, rmc2_blocked),
            "rmc2_to_assembly": shortest_path(self.rack, self.assembly, rmc2_blocked),
            "rmc1_to_assembly": shortest_path(
                self.rmc1_start, self.assembly_approach, rmc1_blocked
            ),
            "rmc1_to_delivery": shortest_path(
                self.assembly_approach, self.delivery_approach, rmc1_blocked
            ),
            "rmc2_return_rack": shortest_path(self.assembly, self.rack, rmc2_blocked),
            "rmc1_return": shortest_path(
                self.delivery_approach, self.rmc1_start, rmc1_blocked
            ),
            "rmc2_return": shortest_path(self.rack, self.rmc2_start, rmc2_blocked),
        }


@dataclass
class Anchor:
    world_x: float
    world_y: float
    world_yaw: float
    odom_x: float
    odom_y: float
    odom_yaw: float

    def to_odom(self, world_x, world_y, world_yaw):
        dx, dy = world_x - self.world_x, world_y - self.world_y
        rotation = self.odom_yaw - self.world_yaw
        cosine, sine = math.cos(rotation), math.sin(rotation)
        return (
            self.odom_x + cosine * dx - sine * dy,
            self.odom_y + sine * dx + cosine * dy,
            wrap_angle(self.odom_yaw + world_yaw - self.world_yaw),
        )


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", help="1/hammer, 2/wrench, 3/pliers")
    parser.add_argument("--rmc1-start", type=int, default=15)
    parser.add_argument("--rmc2-start", type=int, default=0)
    parser.add_argument("--rack", type=int, default=34)
    parser.add_argument("--assembly", type=int, default=16)
    parser.add_argument("--assembly-approach", type=int, default=10)
    parser.add_argument("--delivery", type=int, default=5)
    parser.add_argument("--delivery-approach", type=int, default=4)
    parser.add_argument("--blocked", type=parse_markers, default={31, 35})
    parser.add_argument("--rmc1-start-yaw", type=float, default=math.pi)
    parser.add_argument("--rmc2-start-yaw", type=float, default=math.pi)
    parser.add_argument("--rack-yaw", type=float, default=math.pi)
    parser.add_argument("--auto", action="store_true", help="Не ждать Enter перед стартом")
    parser.add_argument("--dry-run", action="store_true", help="Показать сценарий без ROS")
    parser.add_argument("--skip-arm", action="store_true", help="Только роверы и лифт")
    parser.add_argument("--weights", default="module3/models/latest.pt")
    parser.add_argument(
        "--camera-topic", default="/RMC1/arm95/camera_gripper/image_color"
    )
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--vision-timeout", type=float, default=60.0)
    parser.add_argument("--plane-z", type=float, default=0.128)
    parser.add_argument("--pick-z", type=float, default=0.210)
    parser.add_argument("--approach-z", type=float, default=0.38)
    parser.add_argument("--drop-arm-x", type=float, default=0.55)
    parser.add_argument("--drop-arm-y", type=float, default=0.0)
    parser.add_argument("--drop-arm-z", type=float, default=0.220)
    parser.add_argument("--drop-arm-yaw", type=float, default=math.pi / 2)
    parser.add_argument("--sensor-timeout", type=float, default=3.0)
    parser.add_argument("--waypoint-timeout", type=float, default=35.0)
    args = parser.parse_args()
    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf должен быть от 0 до 1")
    if args.approach_z <= max(args.pick_z, args.drop_arm_z):
        parser.error("--approach-z должен быть выше pick/drop")
    if min(args.sensor_timeout, args.waypoint_timeout, args.vision_timeout) <= 0:
        parser.error("таймауты должны быть положительными")
    return args


def make_scenario(args):
    scenario = Scenario(
        args.rmc1_start,
        args.rmc2_start,
        args.rack,
        args.assembly,
        args.assembly_approach,
        args.delivery,
        args.delivery_approach,
        set(args.blocked),
    )
    scenario.validate()
    return scenario


def print_plan(scenario, routes, target):
    print("\n=== МОДУЛЬ Д: ПЛАН ДО ЗАПУСКА ===")
    print(f"Целевая деталь: {target}")
    print(
        f"RMC2: {scenario.rmc2_start} -> стеллаж {scenario.rack} -> "
        f"комплектация {scenario.assembly} -> {scenario.rack} -> {scenario.rmc2_start}"
    )
    print(
        f"RMC1: {scenario.rmc1_start} -> подход {scenario.assembly_approach} -> "
        f"сдача через {scenario.delivery_approach} -> {scenario.rmc1_start}"
    )
    for name, route in routes.items():
        print(f"{name}: {route}")
    print("Приоритет: STOP -> безопасность -> точность -> скорость")
    print("Команды: stop, resume, abort. Внешний стоп: /mvch/module5/emergency_stop")
    print("====================================\n")


def build_ros_node(args, event_log):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float64, String

    if not rclpy.ok():
        rclpy.init()

    class WarehouseNode(Node):
        def __init__(self):
            super().__init__("mvch_module5")
            self.lock = threading.Lock()
            self.poses = {name: None for name in ROBOT_NAMES}
            self.odom_seen = {name: 0.0 for name in ROBOT_NAMES}
            self.front_distance = {name: math.inf for name in ROBOT_NAMES}
            self.scan_seen = {name: 0.0 for name in ROBOT_NAMES}
            self.last_command = {name: (0.0, 0.0, 0.0, time.monotonic()) for name in ROBOT_NAMES}
            self.emergency = threading.Event()
            self.abort = threading.Event()
            self.lift_status = "unknown"
            self.cmd_publishers = {
                name: self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
                for name in ROBOT_NAMES
            }
            self.lift_publisher = self.create_publisher(Float64, "/RMC2/lift", 10)
            for name in ROBOT_NAMES:
                self.create_subscription(
                    Odometry,
                    f"/{name}/odometry",
                    lambda message, robot=name: self.on_odom(robot, message),
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    LaserScan,
                    f"/{name}/scan_front",
                    lambda message, robot=name: self.on_scan(robot, message),
                    qos_profile_sensor_data,
                )
            self.create_subscription(String, "/RMC2/lift_status", self.on_lift, 10)
            self.create_subscription(Bool, "/mvch/module5/emergency_stop", self.on_stop, 10)
            self.create_subscription(String, "/mvch/module5/command", self.on_command, 10)

        def on_odom(self, robot, message):
            orientation = message.pose.pose.orientation
            siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy = 1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
            pose = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                math.atan2(siny, cosy),
            )
            with self.lock:
                self.poses[robot] = pose
                self.odom_seen[robot] = time.monotonic()

        def on_scan(self, robot, message):
            distances = []
            for index, distance in enumerate(message.ranges):
                angle = message.angle_min + index * message.angle_increment
                if abs(angle) <= math.radians(24) and math.isfinite(distance):
                    if message.range_min <= distance <= message.range_max:
                        distances.append(float(distance))
            with self.lock:
                self.front_distance[robot] = min(distances, default=math.inf)
                self.scan_seen[robot] = time.monotonic()

        def on_lift(self, message):
            self.lift_status = str(message.data)

        def on_stop(self, message):
            if message.data:
                self.emergency.set()
                event_log.write("EMERGENCY_STOP", source="topic")
            else:
                self.emergency.clear()
                event_log.write("RESUME", source="topic")

        def on_command(self, message):
            command = message.data.strip().lower()
            if command == "stop":
                self.emergency.set()
                event_log.write("EMERGENCY_STOP", source="command")
            elif command == "resume":
                self.emergency.clear()
                event_log.write("RESUME", source="command")
            elif command in {"abort", "quit"}:
                self.abort.set()

        def pose(self, robot):
            with self.lock:
                return self.poses[robot], self.odom_seen[robot]

        def scan(self, robot):
            with self.lock:
                return self.front_distance[robot], self.scan_seen[robot]

        def command(self, robot, x=0.0, y=0.0, yaw=0.0, immediate=False):
            now = time.monotonic()
            old_x, old_y, old_yaw, old_time = self.last_command[robot]
            if immediate:
                next_x, next_y, next_yaw = x, y, yaw
            else:
                dt = min(0.15, max(0.02, now - old_time))
                linear_step, angular_step = 0.65 * dt, 1.2 * dt
                next_x = old_x + clamp(x - old_x, linear_step)
                next_y = old_y + clamp(y - old_y, linear_step)
                next_yaw = old_yaw + clamp(yaw - old_yaw, angular_step)
            message = Twist()
            message.linear.x = next_x
            message.linear.y = next_y if robot == "RMC1" else 0.0
            message.angular.z = next_yaw
            self.cmd_publishers[robot].publish(message)
            self.last_command[robot] = (next_x, next_y, next_yaw, now)

        def stop_all(self):
            for _ in range(3):
                for robot in ROBOT_NAMES:
                    self.command(robot, immediate=True)
                time.sleep(0.04)

        def set_lift(self, raised):
            height = 0.1 if raised else 0.0
            event_log.write("LIFT_COMMAND", state="up" if raised else "down", height=height)
            for _ in range(5):
                self.lift_publisher.publish(Float64(data=height))
                time.sleep(0.1)
            started = time.monotonic()
            success = False
            while time.monotonic() - started < 6.0:
                # Предыдущее действие тоже могло оставить status=success.
                # Минимальная пауза соответствует ходу 0.1 м при 0.1 м/с.
                if time.monotonic() - started >= 1.1 and "success" in self.lift_status.lower():
                    success = True
                    break
                time.sleep(0.1)
            if not success:
                raise RuntimeError(f"RMC2: лифт не завершил движение, status={self.lift_status}")
            event_log.write("LIFT_DONE", state="up" if raised else "down", status=self.lift_status)

    return WarehouseNode(), rclpy, MultiThreadedExecutor


class Motion:
    def __init__(self, node, args, event_log, anchors):
        self.node = node
        self.args = args
        self.log = event_log
        self.anchors = anchors

    def wait_ready(self):
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            odometry_ready = all(
                self.node.pose(robot)[0] is not None for robot in ROBOT_NAMES
            )
            lidar_ready = all(self.node.scan(robot)[1] > 0.0 for robot in ROBOT_NAMES)
            if odometry_ready and lidar_ready:
                return
            time.sleep(0.1)
        missing_odom = [robot for robot in ROBOT_NAMES if self.node.pose(robot)[0] is None]
        missing_lidar = [robot for robot in ROBOT_NAMES if self.node.scan(robot)[1] == 0.0]
        raise RuntimeError(
            f"Датчики не готовы; odometry={missing_odom or 'ok'}, "
            f"scan_front={missing_lidar or 'ok'}"
        )

    def wait_if_stopped(self):
        announced = False
        while self.node.emergency.is_set():
            self.node.stop_all()
            if not announced:
                self.log.write("PAUSED")
                announced = True
            if self.node.abort.is_set():
                raise RuntimeError("Остановлено оператором")
            time.sleep(0.1)

    def drive_route(self, robot, name, route, final_world_yaw, carrying=False, rack_entry=False):
        self.log.write("ROUTE", robot=robot, stage=name, markers=route, carrying=carrying)
        for index, marker in enumerate(route[1:], start=1):
            x, y = marker_xy(marker)
            if index == len(route) - 1:
                world_yaw = final_world_yaw
            else:
                nx, ny = marker_xy(route[index + 1])
                world_yaw = math.atan2(ny - y, nx - x)
            target = self.anchors[robot].to_odom(x, y, world_yaw)
            self.drive_to(
                robot,
                marker,
                target,
                carrying=carrying,
                allow_rack_entry=rack_entry and index == len(route) - 1,
            )
        self.log.write("ROUTE_DONE", robot=robot, stage=name, marker=route[-1])

    def drive_to(self, robot, marker, target, carrying=False, allow_rack_entry=False):
        started = time.monotonic()
        obstacle_since = None
        while time.monotonic() - started < self.args.waypoint_timeout:
            if self.node.abort.is_set():
                raise RuntimeError("Остановлено оператором")
            self.wait_if_stopped()
            pose, seen = self.node.pose(robot)
            if pose is None or time.monotonic() - seen > self.args.sensor_timeout:
                self.node.command(robot, immediate=True)
                raise RuntimeError(f"{robot}: потеря odometry")
            x, y, yaw = pose
            tx, ty, target_yaw = target
            dx, dy = tx - x, ty - y
            distance = math.hypot(dx, dy)
            front, scan_seen = self.node.scan(robot)
            fresh_scan = time.monotonic() - scan_seen <= self.args.sensor_timeout
            stop_distance = 0.34 if robot == "RMC2" else 0.30
            rack_zone = allow_rack_entry and distance < 0.72
            blocked = fresh_scan and front < stop_distance and not rack_zone
            if blocked:
                self.node.command(robot, immediate=True)
                if obstacle_since is None:
                    obstacle_since = time.monotonic()
                    self.log.write(
                        "OBSTACLE_STOP", robot=robot, marker=marker, distance=round(front, 3)
                    )
                if time.monotonic() - obstacle_since > 12.0:
                    raise RuntimeError(f"{robot}: препятствие не освободило путь")
                time.sleep(0.08)
                continue
            obstacle_since = None

            yaw_error = wrap_angle(target_yaw - yaw)
            if distance < 0.09:
                if abs(yaw_error) < 0.10:
                    self.node.command(robot, immediate=True)
                    self.log.write(
                        "WAYPOINT", robot=robot, marker=marker, error=round(distance, 3)
                    )
                    return
                self.node.command(robot, yaw=clamp(yaw_error * 1.5, 0.42 if carrying else 0.55))
                time.sleep(0.05)
                continue

            if robot == "RMC1":
                cosine, sine = math.cos(yaw), math.sin(yaw)
                body_x = cosine * dx + sine * dy
                body_y = -sine * dx + cosine * dy
                speed = 0.23 if carrying else 0.32
                self.node.command(
                    robot,
                    x=clamp(body_x * 0.75, speed),
                    y=clamp(body_y * 0.75, speed),
                    yaw=clamp(yaw_error * 1.2, 0.40),
                )
            else:
                heading_error = wrap_angle(math.atan2(dy, dx) - yaw)
                if abs(heading_error) > 0.18:
                    self.node.command(
                        robot, yaw=clamp(heading_error * 1.45, 0.42 if carrying else 0.58)
                    )
                else:
                    maximum = 0.18 if carrying else 0.27
                    self.node.command(
                        robot,
                        x=min(maximum, max(0.07, distance * 0.55)),
                        yaw=clamp(heading_error * 1.2, 0.30),
                    )
            time.sleep(0.05)
        self.node.command(robot, immediate=True)
        raise RuntimeError(f"{robot}: таймаут движения к marker {marker}")


class ArmWorkflow:
    """Тонкая обвязка проверенного зрения и MoveIt из модуля В."""

    def __init__(self, args, event_log, executor, target):
        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        try:
            import cv2
            import numpy as np
            import yaml
            from ament_index_python.packages import get_package_share_directory
            from cv_bridge import CvBridge
            from geometry_msgs.msg import PoseStamped
            from moveit.planning import MoveItPy
            from moveit_configs_utils import MoveItConfigsBuilder
            from rclpy.duration import Duration
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.time import Time
            from sensor_msgs.msg import CameraInfo, Image
            from tf2_ros import Buffer, TransformListener
            from tf_transformations import quaternion_from_euler
            from ultralytics import YOLO
            from module3.module3 import (
                Arm,
                build_vision_node,
                card_near_center,
                choose_target,
                estimate_pose,
                find_targets,
            )
        except ImportError as error:
            raise RuntimeError(
                f"Нет зависимости {error.name}: запустите из module3/.venv "
                "после source ROS или используйте --skip-arm"
            ) from error

        if int(np.__version__.split(".", 1)[0]) >= 2:
            raise RuntimeError("Для cv_bridge нужен numpy<2 в module3/.venv")
        weights = Path(args.weights).expanduser()
        if not weights.is_absolute():
            weights = ROOT / weights
        if not weights.is_file():
            raise RuntimeError(f"Не найдены веса YOLO: {weights}")

        self.args = args
        self.log = event_log
        self.executor = executor
        self.target = target
        self.cv2 = cv2
        self.np = np
        self.find_targets = find_targets
        self.choose_target = choose_target
        self.estimate_pose = estimate_pose
        self.card_near_center = card_near_center
        imports = (
            Node,
            CvBridge,
            Image,
            CameraInfo,
            qos_profile_sensor_data,
            Buffer,
            TransformListener,
            Duration,
            Time,
        )
        self.vision = build_vision_node(SimpleNamespace(topic=args.camera_topic), imports)
        executor.add_node(self.vision)
        self.model = YOLO(str(weights))
        self.log.write("VISION_READY", target=target, weights=str(weights))

        package_dir = Path(get_package_share_directory("ar_webots_fms_ros2"))
        moveit_config = (
            MoveItConfigsBuilder("arm95")
            .robot_description(file_path=str(package_dir / "resource/urdf/arm95_webots.urdf"))
            .trajectory_execution(
                file_path=str(package_dir / "resource/config/moveit_controllers.yaml")
            )
            .moveit_cpp(file_path=str(package_dir / "resource/config/moveit_cpp.yaml"))
            .to_moveit_configs()
            .to_dict()
        )
        moveit_config["use_sim_time"] = True
        with NamedTemporaryFile("w", suffix=".yaml", delete=False) as stream:
            yaml.safe_dump({"/**": {"ros__parameters": moveit_config}}, stream)
            params_file = stream.name
        try:
            self.arm = Arm(
                MoveItPy,
                params_file,
                PoseStamped,
                quaternion_from_euler,
                event_log,
            )
        finally:
            Path(params_file).unlink(missing_ok=True)
        self.log.write("MOVEIT_READY")

    def detect(self):
        started = time.monotonic()
        last_report = 0.0
        seen = {}
        while time.monotonic() - started < self.args.vision_timeout:
            frame = self.vision.latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            result = self.model.predict(
                source=frame,
                conf=self.args.conf,
                imgsz=512,
                device="cpu",
                verbose=False,
            )[0]
            if result.boxes is not None:
                for box in result.boxes:
                    name = str(result.names[int(box.cls.item())]).lower()
                    seen[name] = max(seen.get(name, 0.0), float(box.conf.item()))
            detections = self.find_targets(
                result, frame, self.target, self.cv2, self.np
            )
            detection = self.choose_target(self.vision, detections, self.args.plane_z)
            if detection is not None:
                self.log.write(
                    "DETECTION",
                    target=self.target,
                    confidence=round(detection.confidence, 4),
                )
                return detection
            if time.monotonic() - last_report > 5.0:
                self.log.write(
                    "VISION_WAIT",
                    target=self.target,
                    seen={key: round(value, 3) for key, value in seen.items()},
                )
                last_report = time.monotonic()
        raise RuntimeError(
            f"Класс {self.target} не найден за {self.args.vision_timeout:.0f} с; "
            f"видел: {seen or 'ничего'}"
        )

    def pick(self):
        self.log.write("ARM_STAGE", stage="pick_start")
        self.arm.open()
        self.arm.initial("транспортировочное положение перед захватом")
        detection = self.detect()
        x, y, yaw = self.estimate_pose(self.vision, detection, self.args.plane_z)
        self.log.write("PICK_TARGET", x=round(x, 4), y=round(y, 4), yaw=round(yaw, 4))
        self.arm.pose(x, y, self.args.approach_z, yaw, "подход над деталью")

        # Камера приблизилась: уточняем положение по контуру карточки.
        time.sleep(0.6)
        frame = self.vision.latest_frame()
        if frame is not None:
            try:
                close = self.card_near_center(frame, self.target, self.cv2, self.np)
                refined_x, refined_y, refined_yaw = self.estimate_pose(
                    self.vision, close, self.args.plane_z
                )
                correction = math.hypot(refined_x - x, refined_y - y)
                if correction <= 0.12:
                    x, y, yaw = refined_x, refined_y, refined_yaw
                    self.log.write("PICK_REFINED", correction=round(correction, 4))
            except Exception as error:
                self.log.write("PICK_REFINE_SKIPPED", reason=str(error))

        self.arm.pose(x, y, self.args.approach_z, yaw, "точный подход")
        self.arm.pose(x, y, self.args.pick_z, yaw, "опустить схват")
        self.arm.close()
        self.arm.pose(x, y, self.args.approach_z, yaw, "поднять деталь")
        self.arm.initial("транспортировочное положение с деталью")
        self.log.write("DETAIL_PICKED", target=self.target)

    def place(self):
        self.log.write("ARM_STAGE", stage="place_start")
        x, y = self.args.drop_arm_x, self.args.drop_arm_y
        yaw = self.args.drop_arm_yaw
        self.arm.pose(x, y, self.args.approach_z, yaw, "подход к полке сдачи")
        self.arm.pose(x, y, self.args.drop_arm_z, yaw, "опустить деталь на полку")
        self.arm.open()
        self.arm.pose(x, y, self.args.approach_z, yaw, "отойти от детали")
        self.arm.initial("транспортировочное положение без детали")
        self.log.write("DETAIL_PLACED", target=self.target)

    def close(self):
        try:
            self.executor.remove_node(self.vision)
            self.vision.destroy_node()
        except Exception:
            pass


def terminal_commands(node):
    while not node.abort.is_set():
        try:
            command = input().strip().lower()
        except EOFError:
            return
        if command == "stop":
            node.emergency.set()
        elif command == "resume":
            node.emergency.clear()
        elif command in {"abort", "quit"}:
            node.abort.set()
            return


def resolve_target(value):
    names = {"1": "hammer", "2": "wrench", "3": "pliers"}
    value = (value or "").strip().lower()
    return names.get(value, value)


def run(args):
    target = resolve_target(args.target)
    if not target and not args.dry_run:
        target = resolve_target(input("Цель (1 hammer, 2 wrench, 3 pliers): "))
    if not target:
        target = "hammer"
    if target not in {"hammer", "wrench", "pliers"}:
        raise ValueError(f"Неизвестная цель: {target}")

    scenario = make_scenario(args)
    routes = scenario.routes()
    print_plan(scenario, routes, target)
    if args.dry_run:
        return 0
    if not args.auto:
        input("Проверьте точки и маршруты. Enter — начать: ")

    event_log = EventLog()
    event_log.write("MISSION_START", target=target, log=str(event_log.path))
    mission_started = time.monotonic()
    node, rclpy, MultiThreadedExecutor = build_ros_node(args, event_log)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    exit_code = 0
    arm_workflow = None
    try:
        initial_motion = Motion(node, args, event_log, {})
        initial_motion.wait_ready()
        anchors = {}
        for robot, start_marker, start_yaw in (
            ("RMC1", scenario.rmc1_start, args.rmc1_start_yaw),
            ("RMC2", scenario.rmc2_start, args.rmc2_start_yaw),
        ):
            pose, _ = node.pose(robot)
            world_x, world_y = marker_xy(start_marker)
            anchors[robot] = Anchor(world_x, world_y, start_yaw, *pose)
            event_log.write("LOCALIZED", robot=robot, marker=start_marker, odom=pose)
        motion = Motion(node, args, event_log, anchors)
        threading.Thread(target=terminal_commands, args=(node,), daemon=True).start()

        node.set_lift(False)
        motion.drive_route(
            "RMC2", "to_rack", routes["rmc2_to_rack"], args.rack_yaw, rack_entry=True
        )
        node.set_lift(True)
        motion.drive_route(
            "RMC2",
            "rack_to_assembly",
            routes["rmc2_to_assembly"],
            args.rack_yaw,
            carrying=True,
        )
        node.set_lift(False)

        assembly_yaw = face_marker(scenario.assembly_approach, scenario.assembly)
        motion.drive_route(
            "RMC1", "to_assembly", routes["rmc1_to_assembly"], assembly_yaw
        )
        if args.skip_arm:
            event_log.write("ARM_SKIPPED", stage="pick", reason="--skip-arm")
        else:
            arm_workflow = ArmWorkflow(args, event_log, executor, target)
            arm_workflow.pick()

        delivery_yaw = face_marker(scenario.delivery_approach, scenario.delivery)
        motion.drive_route(
            "RMC1",
            "to_delivery",
            routes["rmc1_to_delivery"],
            delivery_yaw,
            carrying=not args.skip_arm,
        )
        if args.skip_arm:
            event_log.write("ARM_SKIPPED", stage="place", reason="--skip-arm")
        else:
            arm_workflow.place()

        node.set_lift(True)
        motion.drive_route(
            "RMC2",
            "assembly_to_rack",
            routes["rmc2_return_rack"],
            args.rack_yaw,
            carrying=True,
        )
        node.set_lift(False)
        motion.drive_route(
            "RMC1", "return_start", routes["rmc1_return"], args.rmc1_start_yaw
        )
        motion.drive_route(
            "RMC2", "return_start", routes["rmc2_return"], args.rmc2_start_yaw
        )
        event_log.write(
            "MISSION_FINISHED",
            target=target,
            elapsed_seconds=round(time.monotonic() - mission_started, 2),
        )
        print("\nMISSION FINISHED: оба РМК вернулись на старт")
    except KeyboardInterrupt:
        event_log.write("MISSION_ABORTED", reason="Ctrl+C")
        exit_code = 130
    except Exception as error:
        event_log.write("MISSION_FAILED", reason=str(error))
        exit_code = 1
    finally:
        node.abort.set()
        node.stop_all()
        if arm_workflow is not None:
            arm_workflow.close()
        if not executor.shutdown(timeout_sec=2.0):
            event_log.write("SHUTDOWN_TIMEOUT", component="ROS executor")
        # rclpy/MoveItPy Jazzy иногда зависают или падают в C++-деструкторах.
        # Нулевые скорости и логи уже отправлены; ОС безопасно освободит ресурсы.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)
    return exit_code


def main():
    try:
        raise SystemExit(run(arguments()))
    except (ValueError, ImportError) as error:
        raise SystemExit(str(error)) from error


if __name__ == "__main__":
    main()
