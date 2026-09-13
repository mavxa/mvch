#!/usr/bin/env python3
"""Безопасно проехать на RMC по сетке до указанной ArUco-метки.

Скрипт использует штатные ROS 2 топики и TF. По умолчанию рассчитан на
физический робот; для Webots явно передайте --sim.
"""

import argparse
import math
import re
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


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
    """Перенести позу b из системы a в родительскую систему a."""
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


def marker_pose(marker, columns, spacing):
    """Штатная схема ЧВТ: 0=(0,0), 1=(0,+S), COLS=(-S,0)."""
    row, column = divmod(marker, columns)
    return -row * spacing, column * spacing, 0.0


def build_route(start, target, rows, columns, blocked):
    queue = deque([start])
    previous = {start: None}

    while queue:
        current = queue.popleft()
        if current == target:
            break
        row, column = divmod(current, columns)
        for next_row, next_column in (
            (row - 1, column),
            (row + 1, column),
            (row, column - 1),
            (row, column + 1),
        ):
            candidate = next_row * columns + next_column
            if not (0 <= next_row < rows and 0 <= next_column < columns):
                continue
            if candidate in blocked or candidate in previous:
                continue
            previous[candidate] = current
            queue.append(candidate)

    if target not in previous:
        raise RuntimeError(f"Нет маршрута {start} -> {target}")
    route = []
    current = target
    while current is not None:
        route.append(current)
        current = previous[current]
    return list(reversed(route))


class ArucoDrive(Node):
    def __init__(self, args):
        super().__init__("mvch_basic_aruco_drive")
        self.args = args
        self.robot = args.robot.strip("/")
        if args.sim:
            self.set_parameters([Parameter("use_sim_time", value=True)])

        self.odom = None
        self.pose = None
        self.map_from_odom = None
        self.marker = None
        self.marker_error = math.inf
        self.front = None
        self.odom_wall = 0.0
        self.marker_wall = 0.0
        self.scan_wall = 0.0

        self.odom_frame = f"{self.robot}/odom"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if not args.sim:
            tf_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            static_tf_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                TFMessage, f"/{self.robot}/tf", self.on_tf, tf_qos
            )
            self.create_subscription(
                TFMessage,
                f"/{self.robot}/tf_static",
                self.on_tf_static,
                static_tf_qos,
            )
        self.publisher = self.create_publisher(
            Twist, f"/{self.robot}/cmd_vel", 10
        )
        self.create_subscription(
            Odometry,
            f"/{self.robot}/odometry",
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            args.scan_topic or f"/{self.robot}/scan_front",
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.aruco_topic = (
            f"/{self.robot}/aruco_id"
            if args.sim
            else f"/{self.robot}/camera_bottom/aruco_id"
        )
        self.create_subscription(String, self.aruco_topic, self.on_marker, 10)

    def on_tf(self, message):
        for transform in message.transforms:
            self.tf_buffer.set_transform(transform, f"{self.robot} namespaced tf")

    def on_tf_static(self, message):
        for transform in message.transforms:
            self.tf_buffer.set_transform_static(
                transform, f"{self.robot} namespaced tf_static"
            )

    def on_odometry(self, message):
        frame = message.header.frame_id.strip("/")
        if frame:
            self.odom_frame = frame
        pose = message.pose.pose
        values = (pose.position.x, pose.position.y, yaw(pose.orientation))
        if all(math.isfinite(value) for value in values):
            self.odom = values
            self.odom_wall = time.monotonic()

    def on_marker(self, message):
        match = re.fullmatch(r"(?:aruco_)?(\d+)", message.data.strip())
        if match:
            marker = int(match.group(1))
            if 0 <= marker < self.args.rows * self.args.columns:
                self.marker = marker
                self.marker_wall = time.monotonic()

    def on_scan(self, message):
        if not message.ranges or message.angle_increment == 0:
            return
        distances = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) > math.radians(25):
                continue
            if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                distances.append(distance)
        # Пустое пространство lidar часто кодирует как inf. Это свежий скан,
        # просто без препятствий в выбранном секторе.
        self.front = min(distances, default=math.inf)
        self.scan_wall = time.monotonic()

    def localize(self):
        """Привязать odometry к сетке по последней видимой метке."""
        if self.odom is None or self.marker is None:
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
        correction = compose(
            marker_pose(self.marker, self.args.columns, self.args.spacing),
            inverse(odom_marker),
        )
        new_pose = compose(correction, self.odom)
        if self.pose is not None:
            jump = math.hypot(new_pose[0] - self.pose[0], new_pose[1] - self.pose[1])
            if jump > 0.35:
                return
        self.map_from_odom = correction
        self.pose = new_pose
        self.marker_error = math.hypot(
            odom_marker[0] - self.odom[0], odom_marker[1] - self.odom[1]
        )

    def update(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        self.localize()
        if self.map_from_odom is not None and self.odom is not None:
            self.pose = compose(self.map_from_odom, self.odom)

    def stop(self):
        self.publisher.publish(Twist())

    def wait_until_ready(self):
        print("Жду odometry, scan_front, aruco_id и TF метки...")
        deadline = time.monotonic() + self.args.ready_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.update()
            self.stop()
            now = time.monotonic()
            if (
                self.pose is not None
                and self.front is not None
                and now - self.odom_wall < 1.0
                and now - self.scan_wall < 1.0
                and now - self.marker_wall < 1.0
            ):
                return
        raise RuntimeError(
            "Датчики не готовы. Проверьте ros2 topic list, aruco_id и TF aruco_<ID>."
        )

    def check_safety(self):
        now = time.monotonic()
        if now - self.odom_wall > 1.0:
            raise RuntimeError("Потеря odometry")
        if now - self.scan_wall > 1.0:
            raise RuntimeError("Потеря scan_front")
        if self.front < self.args.stop_distance:
            raise RuntimeError(
                f"Препятствие спереди: {self.front:.2f} м "
                f"(< {self.args.stop_distance:.2f} м)"
            )

    def drive_to(self, target):
        target_pose = marker_pose(target, self.args.columns, self.args.spacing)
        deadline = time.monotonic() + self.args.waypoint_timeout

        while rclpy.ok() and time.monotonic() < deadline:
            self.update()
            self.check_safety()
            dx = target_pose[0] - self.pose[0]
            dy = target_pose[1] - self.pose[1]
            distance = math.hypot(dx, dy)
            marker_is_fresh = (
                self.marker == target
                and time.monotonic() - self.marker_wall < 1.0
                and self.marker_error <= self.args.tolerance
            )
            if distance <= self.args.tolerance and marker_is_fresh:
                self.stop()
                print(f"Метка {target} достигнута, ошибка {self.marker_error:.3f} м")
                return

            angle_error = wrap(math.atan2(dy, dx) - self.pose[2])
            command = Twist()
            if abs(angle_error) > 0.15:
                command.angular.z = clamp(
                    1.4 * angle_error,
                    -self.args.max_angular,
                    self.args.max_angular,
                )
            else:
                command.linear.x = min(self.args.max_speed, 0.5 * distance)
                command.angular.z = clamp(
                    1.2 * angle_error,
                    -self.args.max_angular,
                    self.args.max_angular,
                )
            self.publisher.publish(command)

        raise RuntimeError(f"Таймаут движения к метке {target}")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target", nargs="?", type=int, help="ID целевой ArUco")
    parser.add_argument(
        "--target",
        dest="target_option",
        type=int,
        help="ID целевой ArUco (альтернатива позиционному аргументу)",
    )
    parser.add_argument("--robot", choices=("RMC1", "RMC2"), default="RMC2")
    parser.add_argument("--rows", type=int)
    parser.add_argument("--columns", type=int)
    parser.add_argument("--spacing", type=float, default=1.0, help="Шаг сетки, м")
    parser.add_argument("--blocked", type=int, nargs="*", default=[])
    parser.add_argument("--max-speed", type=float, default=0.12, help="м/с")
    parser.add_argument("--max-angular", type=float, default=0.30, help="рад/с")
    parser.add_argument("--stop-distance", type=float, default=0.55, help="м")
    parser.add_argument("--tolerance", type=float, default=0.06, help="м")
    parser.add_argument("--ready-timeout", type=float, default=15.0, help="с")
    parser.add_argument("--waypoint-timeout", type=float, default=45.0, help="с")
    parser.add_argument("--scan-topic", help="Например /RMC2/scan_front")
    parser.add_argument("--yes", action="store_true", help="Не ждать Enter")
    parser.add_argument("--sim", action="store_true", help="Использовать /clock Webots")
    args = parser.parse_args()
    if args.target is not None and args.target_option is not None:
        parser.error("укажите target один раз: позиционно или через --target")
    if args.target is None:
        args.target = args.target_option
    if args.target is None:
        parser.error("укажите ID цели: TARGET или --target TARGET")
    if args.rows is None:
        args.rows = 6 if args.sim else 5
    if args.columns is None:
        args.columns = 6 if args.sim else 5
    if not args.sim and args.robot != "RMC2":
        parser.error(
            "Документация физического RMC1 не содержит нижнюю ArUco-камеру; "
            "drive_to_aruco поддерживает реальный RMC2"
        )
    count = args.rows * args.columns
    if args.rows <= 0 or args.columns <= 0 or not 0 <= args.target < count:
        parser.error(f"target должен быть в диапазоне 0..{count - 1}")
    if args.spacing <= 0 or args.max_speed <= 0 or args.max_angular <= 0:
        parser.error("spacing и скорости должны быть больше нуля")
    return args


def main():
    args = arguments()
    rclpy.init()
    node = ArucoDrive(args)
    exit_code = 0
    try:
        node.wait_until_ready()
        start = node.marker
        blocked = set(args.blocked) - {start, args.target}
        route = build_route(
            start, args.target, args.rows, args.columns, blocked
        )
        print(f"Стартовая метка: {start}")
        print("Маршрут:", " -> ".join(map(str, route)))
        print(
            f"Топики: /{node.robot}/cmd_vel, /{node.robot}/odometry, "
            f"{node.aruco_topic}, {args.scan_topic or f'/{node.robot}/scan_front'}"
        )
        if not args.yes:
            input("Проверьте поле и держите аппаратный STOP. Enter — начать: ")
            # Во время input ROS не обрабатывал сообщения: перед движением снова
            # получить свежие odometry/lidar/ArUco.
            node.wait_until_ready()
        for marker in route[1:]:
            node.drive_to(marker)
        node.stop()
        print(f"FINISHED: ровер стоит над ArUco {args.target}")
    except (KeyboardInterrupt, RuntimeError) as error:
        exit_code = 1
        print(f"STOP: {error}")
    finally:
        for _ in range(10):
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
