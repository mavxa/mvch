#!/usr/bin/env python3
"""Модуль В: распознать деталь, взять ARM95 и вернуть на полку."""

import argparse
import json
import math
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from tempfile import NamedTemporaryFile


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

    def image_path(self, suffix):
        return self.path.with_name(f"{self.path.stem}_{suffix}.jpg")


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
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument(
        "--plane-z",
        type=float,
        default=0.128,
        help="Высота верхней грани детали относительно Base_link, м",
    )
    parser.add_argument("--pick-z", type=float, default=0.210)
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


def find_targets(result, frame, requested, cv2, np):
    found = []
    if result.boxes is None:
        return found
    for box in result.boxes:
        class_id = int(box.cls.item())
        name = str(result.names[class_id]).lower()
        if name != requested:
            continue
        confidence = float(box.conf.item())
        xyxy = tuple(float(value) for value in box.xyxy[0])
        found.append(Detection(name, confidence, xyxy, card_short_axis(frame, xyxy, cv2, np)))
    return found


def choose_target(node, detections, plane_z):
    """При одинаковых классах выбирает ближайшую к базе доступную деталь."""
    reachable = []
    for detection in detections:
        try:
            x, y, _ = estimate_pose(node, detection, plane_z)
        except Exception:
            continue
        reachable.append((math.hypot(x, y), -detection.confidence, detection))
    if reachable:
        return min(reachable, key=lambda item: (item[0], item[1]))[2]
    return max(detections, key=lambda item: item.confidence, default=None)


def card_near_center(frame, requested, cv2, np):
    """На близком кадре находит светлую коробку около центра камеры."""
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 115), (180, 110, 255))
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for contour in contours:
        rectangle = cv2.minAreaRect(contour)
        rect_width, rect_height = rectangle[1]
        rect_area = rect_width * rect_height
        if rect_area < 2000 or min(rect_width, rect_height) < 20:
            continue
        aspect = max(rect_width, rect_height) / min(rect_width, rect_height)
        if not 1.2 <= aspect <= 4.0:
            continue
        center_x, center_y = rectangle[0]
        distance = math.hypot(center_x - width / 2.0, center_y - height / 2.0)
        candidates.append((distance, rectangle))

    if not candidates:
        raise RuntimeError("На близком кадре не найдена коробка")
    distance, rectangle = min(candidates, key=lambda item: item[0])
    if distance > min(width, height) * 0.30:
        raise RuntimeError(f"Коробка слишком далеко от центра кадра: {distance:.0f} px")

    points = cv2.boxPoints(rectangle)
    edges = [points[(index + 1) % 4] - points[index] for index in range(4)]
    short_edge = min(edges, key=lambda edge: float(np.linalg.norm(edge)))
    length = float(np.linalg.norm(short_edge))
    short_axis = float(short_edge[0] / length), float(short_edge[1] / length)
    x1, y1 = np.min(points, axis=0)
    x2, y2 = np.max(points, axis=0)
    return Detection(requested, 1.0, (float(x1), float(y1), float(x2), float(y2)), short_axis)


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
    def __init__(
        self,
        moveit,
        params_file,
        PoseStamped,
        quaternion_from_euler,
        event_log,
        plan_only=False,
    ):
        self.robot = moveit(
            node_name="mvch_module3_moveit",
            name_space="/RMC1/arm95",
            launch_params_filepaths=[params_file],
            config_dict=None,
        )
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
    last_status_time = 0.0
    frame_count = 0
    best_classes = {}
    terminal_trigger = threading.Event()
    target_visible = threading.Event()
    window = not args.no_window and bool(os.environ.get("DISPLAY"))
    if not args.no_window and not window:
        print("[WARN] DISPLAY не найден, работаю без окна")

    def read_terminal_command():
        while not terminal_trigger.is_set():
            try:
                command = input().strip().lower()
            except EOFError:
                return
            if command not in ("", "g", "go"):
                print("[WAIT] Команда не распознана. Введите g и нажмите Enter.")
                continue
            if not target_visible.is_set():
                print(f"[WAIT] Класс {requested} пока не найден, команда не принята.")
                continue
            terminal_trigger.set()

    if not args.auto_start and not args.dry_run and sys.stdin.isatty():
        threading.Thread(target=read_terminal_command, daemon=True).start()

    if args.dry_run:
        print(f"[READY] Dry-run: жду первое распознавание класса {requested}.")
    else:
        print(
            f"[READY] Цель: {requested}. После фиксации нажмите G в окне "
            "или введите g + Enter в терминале."
        )
    while time.monotonic() - started < args.timeout:
        frame = node.latest_frame()
        if frame is None:
            time.sleep(0.05)
            continue
        frame_count += 1
        result = model.predict(
            source=frame,
            conf=args.conf,
            imgsz=args.imgsz,
            device=args.device,
            verbose=False,
        )[0]
        if result.boxes is not None:
            for box in result.boxes:
                name = str(result.names[int(box.cls.item())]).lower()
                confidence = float(box.conf.item())
                best_classes[name] = max(best_classes.get(name, 0.0), confidence)
        detections = find_targets(result, frame, requested, cv2, np)
        last_detection = choose_target(node, detections, args.plane_z)
        if last_detection is None:
            target_visible.clear()
        else:
            target_visible.set()
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
        if last_detection is not None:
            if now - last_frame_time > 0.8:
                event_log.write(
                    "DETECTION",
                    target=last_detection.name,
                    confidence=round(last_detection.confidence, 4),
                    center=[round(value, 1) for value in last_detection.center],
                )
                last_frame_time = now
        elif now - last_status_time > 5.0:
            event_log.write(
                "VISION_WAIT",
                target=requested,
                frames=frame_count,
                seen={name: round(value, 3) for name, value in sorted(best_classes.items())},
            )
            last_status_time = now

        key = -1
        if window:
            cv2.imshow("CHVT module 3 - G: start, ESC: exit", annotated)
            key = cv2.waitKey(1) & 0xFF
        if key == 27:
            raise KeyboardInterrupt
        if last_detection is not None and (
            args.dry_run
            or args.auto_start
            or terminal_trigger.is_set()
            or key in (ord("g"), ord("G"), 13, 32)
        ):
            return last_detection
    if frame_count == 0:
        raise RuntimeError(f"Нет кадров с топика {args.topic} за {args.timeout:.0f} с")
    seen = ", ".join(f"{name}={value:.2f}" for name, value in sorted(best_classes.items()))
    raise RuntimeError(
        f"За {args.timeout:.0f} с класс {requested} не найден; "
        f"обработано кадров: {frame_count}; распознано: {seen or 'ничего'}"
    )


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
    detections = find_targets(result, frame, requested, cv2, np)
    detection = choose_target(node, detections, args.plane_z)
    if detection is None:
        raise RuntimeError(f"После установки руки не найден класс {requested}")
    return detection


def save_snapshot(node, event_log, suffix, cv2):
    time.sleep(0.5)
    frame = node.latest_frame()
    if frame is None:
        event_log.write("SNAPSHOT_FAILED", stage=suffix, reason="нет кадра")
        return
    path = event_log.image_path(suffix)
    if cv2.imwrite(str(path), frame):
        event_log.write("SNAPSHOT", stage=suffix, path=str(path))
    else:
        event_log.write("SNAPSHOT_FAILED", stage=suffix, reason="ошибка записи")


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
        if int(np.__version__.split(".", 1)[0]) >= 2:
            raise SystemExit(
                "NumPy 2.x несовместим с cv_bridge из ROS Jazzy. "
                "Выполните: python3 -m pip install --force-reinstall 'numpy<2'"
            )
        import rclpy
        import yaml
        from ament_index_python.packages import get_package_share_directory
        from cv_bridge import CvBridge
        from geometry_msgs.msg import PoseStamped
        from moveit.planning import MoveItPy
        from moveit_configs_utils import MoveItConfigsBuilder
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
        if error.name == "transforms3d":
            raise SystemExit(
                "Venv не видит системные зависимости ROS. Включите их: "
                "sed -i 's/include-system-site-packages = false/"
                "include-system-site-packages = true/' .venv/pyvenv.cfg"
            )
        raise SystemExit(
            f"Не удалось импортировать {error.name}: {error}. "
            "Выполните pip install -r requirements.txt после source ROS"
        )

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
    moveit_active = False
    exit_code = 0

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
        # Wildcard нужен из-за namespace /RMC1/arm95: обычный config_dict
        # записывает параметры только для имени узла без namespace.
        with NamedTemporaryFile("w", suffix=".yaml", delete=False) as stream:
            yaml.safe_dump({"/**": {"ros__parameters": moveit_config}}, stream)
            params_file = stream.name
        try:
            arm = Arm(
                MoveItPy,
                params_file,
                PoseStamped,
                quaternion_from_euler,
                event_log,
                args.plan_only,
            )
            moveit_active = True
        finally:
            Path(params_file).unlink(missing_ok=True)
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
        time.sleep(0.5)
        close_frame = node.latest_frame()
        if close_frame is None:
            raise RuntimeError("Нет близкого кадра для уточнения")
        close_detection = card_near_center(close_frame, requested, cv2, np)
        close_x, close_y, close_yaw = estimate_pose(node, close_detection, args.plane_z)
        correction = math.hypot(close_x - x, close_y - y)
        if correction > 0.12:
            raise RuntimeError(f"Слишком большая поправка координат: {correction:.3f} м")
        x, y, yaw = close_x, close_y, close_yaw
        event_log.write(
            "TARGET_REFINED",
            method="card_near_center",
            correction=round(correction, 4),
            x=round(x, 4),
            y=round(y, 4),
            yaw=round(yaw, 4),
        )
        arm.pose(x, y, args.approach_z, yaw, "точный подход над деталью")
        arm.pose(x, y, args.pick_z, yaw, "опустить схват")
        arm.close()
        save_snapshot(node, event_log, "gripped", cv2)
        arm.pose(x, y, args.approach_z, yaw, "поднять деталь")
        arm.initial("исходное положение с деталью")
        save_snapshot(node, event_log, "with_detail", cv2)

        arm.pose(drop_x, drop_y, args.approach_z, yaw, "подход к пустому месту")
        arm.pose(drop_x, drop_y, args.pick_z, yaw, "опустить деталь")
        arm.open()
        arm.pose(drop_x, drop_y, args.approach_z, yaw, "отойти от детали")
        arm.initial("исходное положение без детали")
        save_snapshot(node, event_log, "finished", cv2)
        event_log.write("DONE", message="Деталь возвращена, манипулятор в исходном положении")
    except KeyboardInterrupt:
        print("\n[STOP] Остановлено оператором")
        exit_code = 130
    except Exception as error:
        event_log.write("FAILED", reason=str(error))
        exit_code = 1
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if moveit_active:
            # MoveItPy в поставке Jazzy падает в C++-деструкторе после штатной
            # работы. Все движения и логи уже завершены, поэтому не вызываем
            # проблемный teardown при выходе единственного зачетного скрипта.
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(exit_code)
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
