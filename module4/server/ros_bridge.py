#!/usr/bin/env python3
"""Small JSON <-> ROS 2 bridge for the module G web interface.

stdin: one JSON command per line
stdout: one JSON state per line (only machine-readable data)
stderr: diagnostics
"""

import json
import math
import queue
import sys
import threading
import time
from copy import deepcopy

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None


ROBOTS = ("RMC1", "RMC2")
MANUAL_TIMEOUT = 0.35


def empty_robot():
    return {
        "online": False,
        "lastMessageMs": None,
        "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "velocity": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "battery": None,
        "lidar": [],
        "trail": [],
        "plan": [],
        "goal": None,
        "emergency": False,
        "arucoId": None,
        "liftStatus": None,
        "gripper": "unknown",
    }


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class FmsBridge(Node):
    def __init__(self):
        super().__init__("mvch_fms_bridge")
        self.commands = queue.Queue()
        self.lock = threading.Lock()
        self.state = {
            "type": "state",
            "timestamp": int(time.time() * 1000),
            "mode": "ros",
            "bridgeOnline": True,
            "bridgeError": None,
            "map": None,
            "robots": {robot: empty_robot() for robot in ROBOTS},
        }
        self.last_seen = {robot: 0.0 for robot in ROBOTS}
        self.last_manual = {robot: 0.0 for robot in ROBOTS}
        self.last_sent_zero = {robot: True for robot in ROBOTS}
        self.pose_offsets = {robot: {"x": 0.0, "y": 0.0, "yaw": 0.0} for robot in ROBOTS}
        self.raw_pose = {robot: {"x": 0.0, "y": 0.0, "yaw": 0.0} for robot in ROBOTS}
        self.goal_handles = {robot: None for robot in ROBOTS}
        self.simple_goals = {robot: None for robot in ROBOTS}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_publishers = {}
        self.pose_publishers = {}
        self.goal_publishers = {}
        self.nav_clients = {}
        for robot in ROBOTS:
            self.cmd_publishers[robot] = self.create_publisher(Twist, f"/{robot}/cmd_vel", 10)
            self.pose_publishers[robot] = self.create_publisher(
                PoseWithCovarianceStamped, f"/{robot}/initialpose", 10
            )
            self.goal_publishers[robot] = self.create_publisher(PoseStamped, f"/{robot}/goal_pose", 10)
            if NavigateToPose is not None:
                self.nav_clients[robot] = ActionClient(
                    self, NavigateToPose, f"/{robot}/navigate_to_pose"
                )
            self.create_subscription(
                Odometry,
                f"/{robot}/odometry",
                lambda msg, r=robot: self.on_odometry(r, msg),
                sensor_qos,
            )
            self.create_subscription(
                LaserScan,
                f"/{robot}/scan",
                lambda msg, r=robot: self.on_scan(r, msg),
                sensor_qos,
            )
            self.create_subscription(
                Path,
                f"/{robot}/plan",
                lambda msg, r=robot: self.on_path(r, msg),
                10,
            )
            self.create_subscription(
                BatteryState,
                f"/{robot}/battery_state",
                lambda msg, r=robot: self.on_battery(r, msg.voltage),
                sensor_qos,
            )
            self.create_subscription(
                Float64,
                f"/{robot}/battery_voltage",
                lambda msg, r=robot: self.on_battery(r, msg.data),
                sensor_qos,
            )

        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(OccupancyGrid, "/RMC1/map", self.on_map, map_qos)
        self.create_subscription(String, "/RMC2/aruco_id", self.on_aruco, sensor_qos)
        self.create_subscription(String, "/RMC2/lift_status", self.on_lift_status, 10)
        self.lift_publisher = self.create_publisher(Float64, "/RMC2/lift", 10)
        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            "/RMC1/arm95/gripper_trajectory_controller/joint_trajectory",
            10,
        )

        self.create_timer(0.05, self.process_commands_and_watchdog)
        self.create_timer(0.2, self.emit_state)

    def mark_seen(self, robot):
        self.last_seen[robot] = time.monotonic()

    def on_odometry(self, robot, msg):
        raw = {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
        }
        with self.lock:
            self.raw_pose[robot] = raw
            offset = self.pose_offsets[robot]
            pose = {
                "x": raw["x"] + offset["x"],
                "y": raw["y"] + offset["y"],
                "yaw": raw["yaw"] + offset["yaw"],
            }
            rover = self.state["robots"][robot]
            rover["pose"] = pose
            rover["velocity"] = {
                "x": float(msg.twist.twist.linear.x),
                "y": float(msg.twist.twist.linear.y),
                "yaw": float(msg.twist.twist.angular.z),
            }
            trail = rover["trail"]
            if not trail or math.hypot(pose["x"] - trail[-1]["x"], pose["y"] - trail[-1]["y"]) > 0.04:
                trail.append({"x": pose["x"], "y": pose["y"]})
                del trail[:-250]
            self.mark_seen(robot)

    def on_scan(self, robot, msg):
        with self.lock:
            pose = self.state["robots"][robot]["pose"]
            points = []
            stride = max(1, len(msg.ranges) // 180)
            for index in range(0, len(msg.ranges), stride):
                distance = float(msg.ranges[index])
                if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
                    continue
                angle = msg.angle_min + index * msg.angle_increment + pose["yaw"]
                points.append(
                    {
                        "x": pose["x"] + math.cos(angle) * distance,
                        "y": pose["y"] + math.sin(angle) * distance,
                    }
                )
            self.state["robots"][robot]["lidar"] = points[:200]
            self.mark_seen(robot)

    def on_path(self, robot, msg):
        with self.lock:
            self.state["robots"][robot]["plan"] = [
                {"x": float(p.pose.position.x), "y": float(p.pose.position.y)}
                for p in msg.poses[:: max(1, len(msg.poses) // 200)]
            ][:200]

    def on_map(self, msg):
        width, height = int(msg.info.width), int(msg.info.height)
        if not width or not height:
            return
        step = max(1, math.ceil(max(width, height) / 120))
        sampled = []
        for y in range(0, height, step):
            for x in range(0, width, step):
                sampled.append(int(msg.data[y * width + x]))
        with self.lock:
            self.state["map"] = {
                "width": math.ceil(width / step),
                "height": math.ceil(height / step),
                "resolution": float(msg.info.resolution) * step,
                "origin": {
                    "x": float(msg.info.origin.position.x),
                    "y": float(msg.info.origin.position.y),
                },
                "data": sampled,
            }

    def on_battery(self, robot, value):
        if math.isfinite(float(value)):
            with self.lock:
                self.state["robots"][robot]["battery"] = round(float(value), 2)

    def on_aruco(self, msg):
        with self.lock:
            self.state["robots"]["RMC2"]["arucoId"] = str(msg.data)
            self.mark_seen("RMC2")

    def on_lift_status(self, msg):
        with self.lock:
            self.state["robots"]["RMC2"]["liftStatus"] = str(msg.data)

    def publish_twist(self, robot, x=0.0, y=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = max(-0.7, min(0.7, float(x)))
        msg.linear.y = max(-0.7, min(0.7, float(y))) if robot == "RMC1" else 0.0
        msg.angular.z = max(-1.2, min(1.2, float(yaw)))
        self.cmd_publishers[robot].publish(msg)

    def process_commands_and_watchdog(self):
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                break
            try:
                self.handle_command(command)
            except Exception as error:
                print(f"command error: {error}", file=sys.stderr, flush=True)

        now = time.monotonic()
        with self.lock:
            emergency = {r: self.state["robots"][r]["emergency"] for r in ROBOTS}
            simple_goals = deepcopy(self.simple_goals)
        for robot in ROBOTS:
            stale = now - self.last_manual[robot] > MANUAL_TIMEOUT
            if emergency[robot]:
                self.publish_twist(robot)
                self.last_sent_zero[robot] = True
            elif simple_goals[robot] is not None:
                self.drive_simple_goal(robot, simple_goals[robot])
            elif stale and not self.last_sent_zero[robot]:
                self.publish_twist(robot)
                self.last_sent_zero[robot] = True

    def handle_command(self, command):
        kind = command.get("type")
        robot = command.get("robot")
        if robot not in ROBOTS:
            return
        with self.lock:
            emergency = self.state["robots"][robot]["emergency"]

        if kind == "manual" and not emergency:
            self.cancel_navigation(robot)
            with self.lock:
                self.simple_goals[robot] = None
                self.state["robots"][robot]["goal"] = None
                self.state["robots"][robot]["plan"] = []
            self.publish_twist(robot, command.get("x", 0), command.get("y", 0), command.get("yaw", 0))
            self.last_manual[robot] = time.monotonic()
            self.last_sent_zero[robot] = False
        elif kind == "stop":
            self.cancel_navigation(robot)
            with self.lock:
                self.simple_goals[robot] = None
            self.publish_twist(robot)
            self.last_sent_zero[robot] = True
        elif kind == "emergency":
            active = bool(command.get("active"))
            with self.lock:
                self.state["robots"][robot]["emergency"] = active
            self.publish_twist(robot)
            self.last_sent_zero[robot] = True
            if active:
                self.cancel_navigation(robot)
                with self.lock:
                    self.simple_goals[robot] = None
        elif kind == "set_pose":
            self.set_pose(robot, command.get("pose", {}))
        elif kind == "set_goal" and not emergency:
            self.set_goal(robot, command.get("pose", {}))
        elif kind == "cancel_goal":
            self.cancel_navigation(robot)
            self.publish_twist(robot)
            with self.lock:
                self.simple_goals[robot] = None
                self.state["robots"][robot]["goal"] = None
                self.state["robots"][robot]["plan"] = []
        elif kind == "lift" and robot == "RMC2" and not emergency:
            height = 0.1 if float(command.get("height", 0)) >= 0.05 else 0.0
            self.lift_publisher.publish(Float64(data=height))
            with self.lock:
                self.state["robots"][robot]["liftStatus"] = "command: up" if height else "command: down"
        elif kind == "gripper" and robot == "RMC1" and not emergency:
            self.set_gripper(command.get("state") == "open")

    def set_pose(self, robot, pose):
        x, y, yaw = float(pose.get("x", 0)), float(pose.get("y", 0)), float(pose.get("yaw", 0))
        with self.lock:
            raw = self.raw_pose[robot]
            self.pose_offsets[robot] = {"x": x - raw["x"], "y": y - raw["y"], "yaw": yaw - raw["yaw"]}
            self.state["robots"][robot]["pose"] = {"x": x, "y": y, "yaw": yaw}
            self.state["robots"][robot]["trail"] = [{"x": x, "y": y}]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = qx, qy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.pose_publishers[robot].publish(msg)

    def make_goal(self, pose):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(pose.get("x", 0))
        goal.pose.position.y = float(pose.get("y", 0))
        qx, qy, qz, qw = quaternion_from_yaw(float(pose.get("yaw", 0)))
        goal.pose.orientation.x, goal.pose.orientation.y = qx, qy
        goal.pose.orientation.z, goal.pose.orientation.w = qz, qw
        return goal

    def set_goal(self, robot, pose):
        goal = self.make_goal(pose)
        self.goal_publishers[robot].publish(goal)
        target = {"x": goal.pose.position.x, "y": goal.pose.position.y, "yaw": float(pose.get("yaw", 0))}
        with self.lock:
            rover = self.state["robots"][robot]
            rover["goal"] = target
            if not rover["plan"]:
                rover["plan"] = [
                    {"x": rover["pose"]["x"], "y": rover["pose"]["y"]},
                    {"x": target["x"], "y": target["y"]},
                ]
        client = self.nav_clients.get(robot)
        if client is not None and client.server_is_ready():
            request = NavigateToPose.Goal()
            request.pose = goal
            future = client.send_goal_async(request)
            future.add_done_callback(lambda f, r=robot: self.on_goal_response(r, f))
        else:
            with self.lock:
                self.simple_goals[robot] = target
            print(f"{robot}: Nav2 is unavailable; using odometry goal controller", file=sys.stderr, flush=True)

    def drive_simple_goal(self, robot, target):
        with self.lock:
            pose = deepcopy(self.state["robots"][robot]["pose"])
        dx = target["x"] - pose["x"]
        dy = target["y"] - pose["y"]
        distance = math.hypot(dx, dy)

        if distance < 0.12:
            yaw_error = wrap_angle(target["yaw"] - pose["yaw"])
            if abs(yaw_error) < 0.12:
                self.publish_twist(robot)
                with self.lock:
                    self.simple_goals[robot] = None
                self.last_sent_zero[robot] = True
            else:
                self.publish_twist(robot, yaw=clamp(yaw_error * 1.5, 0.65))
                self.last_sent_zero[robot] = False
            return

        if robot == "RMC1":
            cosine, sine = math.cos(pose["yaw"]), math.sin(pose["yaw"])
            body_x = cosine * dx + sine * dy
            body_y = -sine * dx + cosine * dy
            self.publish_twist(
                robot,
                x=clamp(body_x * 0.7, 0.38),
                y=clamp(body_y * 0.7, 0.38),
                yaw=clamp(wrap_angle(target["yaw"] - pose["yaw"]), 0.45),
            )
        else:
            heading_error = wrap_angle(math.atan2(dy, dx) - pose["yaw"])
            if abs(heading_error) > 0.28:
                self.publish_twist(robot, yaw=clamp(heading_error * 1.4, 0.7))
            else:
                self.publish_twist(
                    robot,
                    x=min(0.32, max(0.10, distance * 0.65)),
                    yaw=clamp(heading_error * 1.2, 0.45),
                )
        self.last_sent_zero[robot] = False

    def on_goal_response(self, robot, future):
        handle = future.result()
        if handle.accepted:
            self.goal_handles[robot] = handle
        else:
            print(f"{robot}: Nav2 rejected goal", file=sys.stderr, flush=True)

    def cancel_navigation(self, robot):
        handle = self.goal_handles.get(robot)
        if handle is not None:
            handle.cancel_goal_async()
            self.goal_handles[robot] = None

    def set_gripper(self, opened):
        msg = JointTrajectory()
        msg.joint_names = ["left_joint", "right_joint"]
        point = JointTrajectoryPoint()
        value = 0.04 if opened else 0.0213
        point.positions = [value, value]
        point.time_from_start = Duration(sec=1)
        msg.points = [point]
        self.gripper_publisher.publish(msg)
        with self.lock:
            self.state["robots"]["RMC1"]["gripper"] = "open" if opened else "closed"

    def emit_state(self):
        now = time.monotonic()
        with self.lock:
            for robot in ROBOTS:
                age = now - self.last_seen[robot]
                self.state["robots"][robot]["online"] = age < 2.0
                self.state["robots"][robot]["lastMessageMs"] = round(age * 1000) if self.last_seen[robot] else None
            self.state["timestamp"] = int(time.time() * 1000)
            payload = deepcopy(self.state)
        print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")), flush=True)


def read_commands(node):
    for line in sys.stdin:
        try:
            command = json.loads(line)
            if isinstance(command, dict):
                node.commands.put(command)
        except json.JSONDecodeError as error:
            print(f"invalid command JSON: {error}", file=sys.stderr, flush=True)


def main():
    rclpy.init()
    node = FmsBridge()
    threading.Thread(target=read_commands, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            for robot in ROBOTS:
                node.publish_twist(robot)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
