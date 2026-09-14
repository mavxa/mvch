#!/usr/bin/env python3
import argparse
import json
import math
import re
import time
from collections import deque
from datetime import datetime
from pathlib import Path


ROWS = 5
COLUMNS = 5
SPACING = 1.0
MAX_SPEED = 0.35
MAX_ANGULAR = 0.60
STOP_DISTANCE = 0.65
TOLERANCE = 0.07
SCAN_TOPIC = "/RMC2/scan_front"
BLOCKED = set()


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(quaternion):
    return math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y**2 + quaternion.z**2),
    )


def compose(a, b):
    cosine, sine = math.cos(a[2]), math.sin(a[2])
    return (
        a[0] + cosine * b[0] - sine * b[1],
        a[1] + sine * b[0] + cosine * b[1],
        wrap(a[2] + b[2]),
    )


def inverse(pose):
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    return (
        -cosine * pose[0] - sine * pose[1],
        sine * pose[0] - cosine * pose[1],
        -pose[2],
    )


def marker_pose(marker):
    row, column = divmod(marker, COLUMNS)
    return -row * SPACING, column * SPACING, 0.0


def shortest_path(start, goal, forbidden=()):
    if start in BLOCKED or goal in BLOCKED:
        raise RuntimeError("невозможная цель")

    forbidden = {frozenset(edge) for edge in forbidden}
    search = deque([start])
    parent = {start: None}

    while search:
        current = search.popleft()
        if current == goal:
            break
        row, column = divmod(current, COLUMNS)
        for next_row, next_column in (
            (row - 1, column),
            (row + 1, column),
            (row, column - 1),
            (row, column + 1),
        ):
            if not (0 <= next_row < ROWS and 0 <= next_column < COLUMNS):
                continue
            candidate = next_row * COLUMNS + next_column
            edge = frozenset((current, candidate))
            if candidate in BLOCKED or candidate in parent or edge in forbidden:
                continue
            parent[candidate] = current
            search.append(candidate)

    if goal not in parent:
        raise RuntimeError(f"нет маршрута {start} к {goal}")

    route = []
    current = goal
    while current is not None:
        route.append(current)
        current = parent[current]
    return list(reversed(route))


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=int, required=True)
    return parser.parse_args().target


def main():
    target_id = arguments()

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException, TransformListener

    class Rover(Node):
        def __init__(self):
            super().__init__("mvch_module_b")
            self.cmd = self.create_publisher(Twist, "/RMC2/cmd_vel", 10)
            self.create_subscription(
                Odometry, "/RMC2/odometry", self.on_odom, qos_profile_sensor_data
            )
            self.create_subscription(
                LaserScan, SCAN_TOPIC, self.on_scan, qos_profile_sensor_data
            )
            self.create_subscription(
                String, "/RMC2/camera_bottom/aruco_id", self.on_marker, 10
            )
            self.create_subscription(String, "/RMC2/aruco_id", self.on_marker, 10)

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            tf_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            static_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(TFMessage, "/RMC2/tf", self.on_tf, tf_qos)
            self.create_subscription(
                TFMessage, "/RMC2/tf_static", self.on_tf_static, static_qos
            )

            self.odom = None
            self.odom_frame = "RMC2/odom"
            self.map_from_odom = None
            self.pose = None
            self.marker = None
            self.marker_seen = 0.0
            self.marker_error = math.inf
            self.front = math.inf
            self.current = None
            self.leg = "outbound"

            reports = Path(__file__).parent / "reports"
            reports.mkdir(exist_ok=True)
            self.log_path = reports / f"module_b_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
            self.log_file = self.log_path.open("a", encoding="utf-8", buffering=1)

        def log(self, event, **data):
            item = {
                "time": datetime.now().isoformat(timespec="milliseconds"),
                "event": event,
                "leg": self.leg,
                **data,
            }
            line = json.dumps(item, ensure_ascii=False)
            print(line, flush=True)
            self.log_file.write(line + "\n")

        def on_tf(self, message):
            for transform in message.transforms:
                self.tf_buffer.set_transform(transform, "RMC2 tf")

        def on_tf_static(self, message):
            for transform in message.transforms:
                self.tf_buffer.set_transform_static(transform, "RMC2 tf_static")

        def on_odom(self, message):
            frame = message.header.frame_id.strip("/")
            if frame:
                self.odom_frame = frame
            pose = message.pose.pose
            self.odom = pose.position.x, pose.position.y, yaw(pose.orientation)

        def on_marker(self, message):
            match = re.search(r"\d+", message.data)
            if not match:
                return
            marker = int(match.group())
            if not 0 <= marker < ROWS * COLUMNS:
                return
            if marker != self.marker:
                self.log("MARKER", marker=marker)
            self.marker = marker
            self.marker_seen = time.monotonic()

        def on_scan(self, message):
            front = []
            for index, value in enumerate(message.ranges):
                angle = message.angle_min + index * message.angle_increment
                if abs(angle) <= math.radians(25):
                    if math.isfinite(value) and message.range_min <= value <= message.range_max:
                        front.append(value)
            self.front = min(front, default=math.inf)

        def localize(self):
            if self.odom is None or self.marker is None:
                return
            if time.monotonic() - self.marker_seen > 0.8:
                return
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.odom_frame, f"aruco_{self.marker}", Time()
                ).transform
            except TransformException:
                return

            odom_marker = (
                transform.translation.x,
                transform.translation.y,
                yaw(transform.rotation),
            )
            self.map_from_odom = compose(
                marker_pose(self.marker), inverse(odom_marker)
            )
            self.pose = compose(self.map_from_odom, self.odom)
            self.marker_error = math.hypot(
                odom_marker[0] - self.odom[0], odom_marker[1] - self.odom[1]
            )

        def update(self):
            rclpy.spin_once(self, timeout_sec=0.05)
            self.localize()
            if self.map_from_odom is not None and self.odom is not None:
                self.pose = compose(self.map_from_odom, self.odom)

        def stop(self):
            self.cmd.publish(Twist())

        def send(self, linear, angular):
            command = Twist()
            command.linear.x = float(linear)
            command.angular.z = float(angular)
            self.cmd.publish(command)

        def wait_start(self):
            print("Жду ArUco под ровером. Старт определится автоматически.")
            while rclpy.ok():
                self.update()
                self.stop()
                if self.pose is not None and self.marker_error <= TOLERANCE:
                    self.current = self.marker
                    self.log("START_FOUND", marker=self.current)
                    return self.current

        def reached(self, marker):
            return (
                self.marker == marker
                and time.monotonic() - self.marker_seen < 0.8
                and self.marker_error <= TOLERANCE
            )

        def drive_to(self, marker, check_obstacle=True):
            target = marker_pose(marker)
            while rclpy.ok():
                self.update()
                if self.pose is None:
                    self.stop()
                    continue
                if self.reached(marker):
                    self.stop()
                    return True

                dx = target[0] - self.pose[0]
                dy = target[1] - self.pose[1]
                distance = math.hypot(dx, dy)
                error = wrap(math.atan2(dy, dx) - self.pose[2])
                linear = 0.0 if abs(error) > 0.16 else min(MAX_SPEED, max(0.04, 0.7 * distance))
                angular = clamp(1.4 * error, -MAX_ANGULAR, MAX_ANGULAR)

                if check_obstacle and linear > 0 and self.front < STOP_DISTANCE:
                    self.stop()
                    return False
                self.send(linear, angular)

        def follow(self, goal, forbidden):
            route = shortest_path(self.current, goal, forbidden)
            index = 1
            while index < len(route):
                next_marker = route[index]
                if self.drive_to(next_marker):
                    self.current = next_marker
                    self.log("WAYPOINT", marker=self.current)
                    index += 1
                    continue

                edge = frozenset((self.current, next_marker))
                forbidden.add(edge)
                self.log(
                    "OBSTACLE",
                    edge=sorted(edge),
                    distance=round(self.front, 3),
                )
                print(f"Препятствие перед {next_marker}. Возвращаюсь к {self.current}.")
                self.drive_to(self.current, check_obstacle=False)
                route = shortest_path(self.current, goal, forbidden)
                index = 1
                show_route(self, "НОВЫЙ МАРШРУТ", route, "ROUTE_REPLANNED")
            return route

    def show_route(rover, title, route, event):
        print(f"{title}: {' путь: '.join(map(str, route))}")
        rover.log(event, markers=route)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    rover = Rover()
    exit_code = 0
    try:
        start = rover.wait_start()
        route_there = shortest_path(start, target_id)
        route_back = shortest_path(target_id, start)

        print()
        show_route(rover, "МАРШРУТ К ЦЕЛИ", route_there, "ROUTE_TO_TARGET")
        show_route(rover, "МАРШРУТ НА СТАРТ", route_back, "ROUTE_TO_START")
        input("\nenter - начать движение:")

        started = time.monotonic()
        rover.log("MOVEMENT_START", markers=route_there)
        rover.follow(target_id, set())
        first_time = time.monotonic() - started
        rover.log("MOVEMENT_FINISHED", marker=target_id, seconds=round(first_time, 2))
        rover.log("TARGET_REACHED", marker=target_id, seconds=round(first_time, 2))

        input("\nцель достигнута. ожидание препятствие и enter: ")
        rover.leg = "return"
        show_route(rover, "МАРШРУТ НА СТАРТ", route_back, "ROUTE_TO_START")
        input("Enter - начать возврат: ")

        started = time.monotonic()
        rover.log("MOVEMENT_START", markers=route_back)
        rover.follow(start, set())
        second_time = time.monotonic() - started
        rover.stop()
        rover.log("MOVEMENT_FINISHED", marker=start, seconds=round(second_time, 2))
        rover.log("START_REACHED", marker=start, seconds=round(second_time, 2))
        rover.log(
            "MISSION_FINISHED",
            marker=start,
            movement_seconds=round(first_time + second_time, 2),
            log=str(rover.log_path),
        )
    except (KeyboardInterrupt, RuntimeError) as error:
        exit_code = 1
        rover.log("FAILED", reason=str(error) or "Ctrl+C")
    finally:
        for _ in range(5):
            rover.stop()
            rclpy.spin_once(rover, timeout_sec=0.02)
        rover.log_file.close()
        rover.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
