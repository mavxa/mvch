#!/usr/bin/env python3
"""Модуль В: распознать деталь, взять ARM95 и вернуть на полку."""

import argparse
import json
import math
import os
import threading
import time
from datetime import datetime
from pathlib import Path


HERE = Path(__file__).resolve().parent
TARGETS = {"1": "hammer", "2": "wrench", "3": "pliers"}


class EventLog:
    def __init__(self):
        directory = HERE / "logs"
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f"module3_{datetime.now():%Y%m%d_%H%M%S}.log"

    def write(self, event, **fields):
        record = {"time": datetime.now().isoformat(timespec="milliseconds"), "event": event}
        record.update(fields)
        line = json.dumps(record, ensure_ascii=False)
        print(line, flush=True)
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")


def local_path(value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else HERE / path


def target_name(value):
    value = value.strip().lower()
    return TARGETS.get(value, value)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", help="1/hammer, 2/wrench или 3/pliers")
    parser.add_argument("--weights", default="models/latest.pt")
    parser.add_argument("--topic", default="/RMC1/arm95/camera_gripper/image_color")
    parser.add_argument("--conf", type=float, default=0.45)
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument(
        "--plane-z",
        type=float,
        default=0.128,
        help="Высота верхней грани детали относительно Base_link, м",
    )
    parser.add_argument("--pick-z", type=float, default=0.235)
    parser.add_argument("--approach-z", type=float, default=0.38)
    parser.add_argument("--drop-x", type=float, help="Точка возврата; по умолчанию исходная")
    parser.add_argument("--drop-y", type=float, help="Точка возврата; по умолчанию исходная")
    parser.add_argument("--no-window", action="store_true", help="Не открывать видеопоток")
    parser.add_argument("--auto-start", action="store_true", help="Не ждать клавишу G/Enter")
    parser.add_argument("--dry-run", action="store_true", help="Только распознавание и координаты")
    parser.add_argument("--plan-only", action="store_true", help="Спланировать подход, не двигать руку")
    args = parser.parse_args()

    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf должен быть от 0 до 1")
    if args.timeout <= 0 or args.approach_z <= args.pick_z:
        parser.error("timeout > 0, approach-z должен быть выше pick-z")
    if (args.drop_x is None) != (args.drop_y is None):
        parser.error("--drop-x и --drop-y задаются вместе")
    return args


def rotate_vector(vector, quaternion):
    """Поворот вектора кватернионом geometry_msgs без внешней математики."""
    x, y, z = vector
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def normalize_half_turn(angle, reference=math.pi / 2):
    """Выбирает эквивалентный yaw (схват симметричен через 180 градусов)."""
    choices = (angle - math.pi, angle, angle + math.pi)
    return min(choices, key=lambda item: abs(item - reference))


def card_short_axis(frame, xyxy, cv2, np):
    """Находит направление короткой стороны светлой карточки внутри YOLO bbox."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = (int(round(value)) for value in xyxy)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(width, x2), min(height, y2)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 105), (180, 115, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    crop_area = crop.shape[0] * crop.shape[1]
    candidates = [contour for contour in contours if cv2.contourArea(contour) > crop_area * 0.12]
    if not candidates:
        return None
    rectangle = cv2.minAreaRect(max(candidates, key=cv2.contourArea))
    points = cv2.boxPoints(rectangle)
    edges = [points[(index + 1) % 4] - points[index] for index in range(4)]
    short_edge = min(edges, key=lambda edge: float(np.linalg.norm(edge)))
    length = float(np.linalg.norm(short_edge))
    if length < 2.0:
        return None
    return float(short_edge[0] / length), float(short_edge[1] / length)


class Detection:
    def __init__(self, name, confidence, xyxy, short_axis):
        self.name = name
        self.confidence = confidence
        self.xyxy = xyxy
        self.short_axis = short_axis

    @property
    def center(self):
        x1, y1, x2, y2 = self.xyxy
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def find_target(result, frame, requested, cv2, np):
    found = []
    if result.boxes is None:
        return None
    for box in result.boxes:
        class_id = int(box.cls.item())
        name = str(result.names[class_id]).lower()
        if name != requested:
            continue
        confidence = float(box.conf.item())
        xyxy = tuple(float(value) for value in box.xyxy[0])
        found.append(Detection(name, confidence, xyxy, card_short_axis(frame, xyxy, cv2, np)))
    return max(found, key=lambda item: item.confidence, default=None)


def estimate_pose(node, detection, plane_z):
    if node.camera_info is None:
        raise RuntimeError("Нет CameraInfo")
    frame = node.camera_frame or node.camera_info.header.frame_id
    transform = node.tf_buffer.lookup_transform(
        "Base_link", frame, node.Time(), timeout=node.Duration(seconds=1.0)
    )
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)

    u, v = detection.center
    matrix = node.camera_info.k
    fx, fy, cx, cy = matrix[0], matrix[4], matrix[2], matrix[5]
    ray_camera = ((u - cx) / fx, (v - cy) / fy, 1.0)
    ray_base = rotate_vector(ray_camera, quaternion)
    if abs(ray_base[2]) < 1e-6:
        raise RuntimeError("Луч камеры параллелен полке")
    distance = (plane_z - translation.z) / ray_base[2]
    if distance <= 0:
        raise RuntimeError("Полка находится позади камеры")

    x = translation.x + distance * ray_base[0]
    y = translation.y + distance * ray_base[1]
    if math.hypot(x, y) > 0.9:
        raise RuntimeError(f"Координаты вне рабочей зоны: x={x:.3f}, y={y:.3f}")

    yaw = math.pi / 2
    if detection.short_axis is not None:
        vx, vy = detection.short_axis
        axis_base = rotate_vector((vx, vy, 0.0), quaternion)
        short_angle = math.atan2(axis_base[1], axis_base[0])
        yaw = normalize_half_turn(short_angle - math.pi / 2)
    return x, y, yaw


def build_vision_node(args, imports):
    Node, CvBridge, Image, CameraInfo, qos, Buffer, TransformListener, Duration, Time = imports

    class VisionNode(Node):
        def __init__(self):
            super().__init__("mvch_module3_vision")
            self.Duration = Duration
            self.Time = Time
            self.bridge = CvBridge()
            self.frame = None
            self.camera_info = None
            self.camera_frame = None
            self.lock = threading.Lock()
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(Image, args.topic, self.on_image, qos)
            info_topic = args.topic.rsplit("/", 1)[0] + "/camera_info"
            self.create_subscription(CameraInfo, info_topic, self.on_info, qos)

        def on_image(self, message):
            try:
                frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            except Exception as error:
                self.get_logger().error(f"Ошибка кадра: {error}")
                return
            with self.lock:
                self.frame = frame
                self.camera_frame = message.header.frame_id

        def on_info(self, message):
            self.camera_info = message

        def latest_frame(self):
            with self.lock:
                return None if self.frame is None else self.frame.copy()

    return VisionNode()


class Arm:
    def __init__(self, moveit, PoseStamped, quaternion_from_euler, event_log, plan_only=False):
        self.robot = moveit(node_name="mvch_module3_moveit", name_space="/RMC1/arm95")
        self.arm = self.robot.get_planning_component("arm95_group")
        self.gripper = self.robot.get_planning_component("gripper")
        self.PoseStamped = PoseStamped
        self.quaternion_from_euler = quaternion_from_euler
        self.log = event_log
        self.plan_only = plan_only

    def execute(self, component, stage):
        plan = component.plan()
        if not plan:
            raise RuntimeError(f"MoveIt не построил траекторию: {stage}")
        self.log.write("PLAN_READY", stage=stage)
        if not self.plan_only:
            self.robot.execute(plan.trajectory, controllers=[])
            time.sleep(0.4)
            self.log.write("MOTION_DONE", stage=stage)

    def named(self, component, state, stage):
        component.set_start_state_to_current_state()
        component.set_goal_state(configuration_name=state)
        self.execute(component, stage)

    def pose(self, x, y, z, yaw, stage):
        goal = self.PoseStamped()
        goal.header.frame_id = "Base_link"
        qx, qy, qz, qw = self.quaternion_from_euler(math.pi, 0.0, yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=goal, pose_link="gripper_base")
        self.execute(self.arm, stage)

    def open(self):
        self.named(self.gripper, "open", "открыть схват")
        if not self.plan_only:
            self.log.write("GRIPPER", state="open")

    def close(self):
        self.named(self.gripper, "closed", "закрыть схват")
        if not self.plan_only:
            self.log.write("GRIPPER", state="closed")

    def initial(self, stage="исходное положение"):
        self.named(self.arm, "table_pos", stage)


def wait_for_trigger(args, node, model, requested, cv2, np, event_log):
    started = time.monotonic()
    last_detection = None
    last_frame_time = 0.0
    window = not args.no_window and bool(os.environ.get("DISPLAY"))
    if not args.no_window and not window:
        print("[WARN] DISPLAY не найден, работаю без окна")

    print(f"[READY] Цель: {requested}. После фиксации экспертом нажмите G в окне.")
    while time.monotonic() - started < args.timeout:
        frame = node.latest_frame()
        if frame is None:
            time.sleep(0.05)
            continue
        result = model.predict(
            source=frame,
            conf=args.conf,
            imgsz=args.imgsz,
            device=args.device,
            verbose=False,
        )[0]
        last_detection = find_target(result, frame, requested, cv2, np)
        annotated = result.plot(labels=True, conf=True, line_width=2)
        if last_detection is not None:
            u, v = (int(value) for value in last_detection.center)
            cv2.circle(annotated, (u, v), 7, (0, 255, 255), -1)
            if last_detection.short_axis is not None:
                vx, vy = last_detection.short_axis
                cv2.line(
                    annotated,
                    (u - int(vx * 45), v - int(vy * 45)),
                    (u + int(vx * 45), v + int(vy * 45)),
                    (255, 255, 0),
                    3,
                )

        now = time.monotonic()
        if last_detection is not None and now - last_frame_time > 0.8:
            event_log.write(
                "DETECTION",
                target=last_detection.name,
                confidence=round(last_detection.confidence, 4),
                center=[round(value, 1) for value in last_detection.center],
            )
            last_frame_time = now

        key = -1
        if window:
            cv2.imshow("CHVT module 3 - G: start, ESC: exit", annotated)
            key = cv2.waitKey(1) & 0xFF
        if key == 27:
            raise KeyboardInterrupt
        if last_detection is not None and (args.auto_start or key in (ord("g"), ord("G"), 13, 32)):
            return last_detection
        if not window and last_detection is not None and not args.auto_start:
            input("Цель найдена. После фиксации экспертом нажмите Enter: ")
            return last_detection
    raise RuntimeError(f"За {args.timeout:.0f} с класс {requested} не найден")


def fresh_detection(node, model, requested, args, cv2, np):
    time.sleep(0.7)
    frame = node.latest_frame()
    if frame is None:
        raise RuntimeError("Нет свежего кадра после возврата руки")
    result = model.predict(
        source=frame,
        conf=args.conf,
        imgsz=args.imgsz,
        device=args.device,
        verbose=False,
    )[0]
    detection = find_target(result, frame, requested, cv2, np)
    if detection is None:
        raise RuntimeError(f"После установки руки не найден класс {requested}")
    return detection


def main():
    args = arguments()
    requested = target_name(args.target or input("Цель (1 hammer, 2 wrench, 3 pliers): "))
    if requested not in TARGETS.values():
        raise SystemExit(f"Неизвестная цель: {requested}")
    weights = local_path(args.weights)
    if not weights.is_file():
        raise SystemExit(f"Не найдены веса: {weights}")

    try:
        import cv2
        import numpy as np
        import rclpy
        from cv_bridge import CvBridge
        from geometry_msgs.msg import PoseStamped
        from moveit.planning import MoveItPy
        from rclpy.duration import Duration
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import Buffer, TransformListener
        from tf_transformations import quaternion_from_euler
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit(f"Нет зависимости {error.name}: запускайте из module3/.venv после source ROS")

    event_log = EventLog()
    event_log.write("START", target=requested, log=str(event_log.path))
    rclpy.init()
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
    node = build_vision_node(args, imports)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    model = YOLO(str(weights))

    try:
        detection = wait_for_trigger(args, node, model, requested, cv2, np, event_log)
        if args.dry_run:
            x, y, yaw = estimate_pose(node, detection, args.plane_z)
            event_log.write(
                "DRY_RUN",
                x=round(x, 4),
                y=round(y, 4),
                plane_z=args.plane_z,
                yaw=round(yaw, 4),
            )
            return

        arm = Arm(MoveItPy, PoseStamped, quaternion_from_euler, event_log, args.plan_only)
        arm.open()
        arm.initial("установить исходное положение")
        detection = fresh_detection(node, model, requested, args, cv2, np)
        x, y, yaw = estimate_pose(node, detection, args.plane_z)
        drop_x = x if args.drop_x is None else args.drop_x
        drop_y = y if args.drop_y is None else args.drop_y
        event_log.write(
            "TARGET",
            target=requested,
            confidence=round(detection.confidence, 4),
            x=round(x, 4),
            y=round(y, 4),
            yaw=round(yaw, 4),
        )

        arm.pose(x, y, args.approach_z, yaw, "подход над деталью")
        if args.plan_only:
            print("[PLAN-ONLY] Подход построен, движения не было")
            return
        arm.pose(x, y, args.pick_z, yaw, "опустить схват")
        arm.close()
        arm.pose(x, y, args.approach_z, yaw, "поднять деталь")
        arm.initial("исходное положение с деталью")

        arm.pose(drop_x, drop_y, args.approach_z, yaw, "подход к пустому месту")
        arm.pose(drop_x, drop_y, args.pick_z, yaw, "опустить деталь")
        arm.open()
        arm.pose(drop_x, drop_y, args.approach_z, yaw, "отойти от детали")
        arm.initial("исходное положение без детали")
        event_log.write("DONE", message="Деталь возвращена, манипулятор в исходном положении")
    except KeyboardInterrupt:
        print("\n[STOP] Остановлено оператором")
    except Exception as error:
        event_log.write("FAILED", reason=str(error))
        raise SystemExit(1)
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
