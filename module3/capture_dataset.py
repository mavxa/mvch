#!/usr/bin/env python3
"""Сохраняет непохожие кадры с камеры захвата RMC1 для разметки."""

import argparse
import time
from datetime import datetime
from pathlib import Path


HERE = Path(__file__).resolve().parent


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic",
        default="/RMC1/arm95/camera_gripper/image_color",
        help="ROS-топик sensor_msgs/Image",
    )
    parser.add_argument("--output", default="dataset/raw", help="Каталог для JPG")
    parser.add_argument("--count", type=int, default=30, help="Сколько кадров сохранить")
    parser.add_argument("--interval", type=float, default=0.5, help="Минимум секунд между JPG")
    parser.add_argument(
        "--min-change",
        type=float,
        default=1.0,
        help="Минимальное среднее изменение пикселя; 0 сохраняет одинаковые кадры",
    )
    parser.add_argument("--timeout", type=float, default=180.0, help="Общий таймаут")
    parser.add_argument("--jpeg-quality", type=int, default=95)
    args = parser.parse_args()
    if args.count < 1 or args.interval < 0 or args.timeout <= 0:
        parser.error("count >= 1, interval >= 0, timeout > 0")
    if not 1 <= args.jpeg_quality <= 100 or args.min_change < 0:
        parser.error("jpeg-quality должен быть 1..100, min-change >= 0")
    return args


def main():
    args = arguments()
    try:
        import cv2
        import numpy as np
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as error:
        raise SystemExit(
            f"Нет ROS/OpenCV зависимости: {error}. Запускайте после source ROS 2 в VM."
        )

    output = Path(args.output).expanduser()
    if not output.is_absolute():
        output = HERE / output
    output.mkdir(parents=True, exist_ok=True)

    class Collector(Node):
        def __init__(self):
            super().__init__("mvch_dataset_collector")
            self.bridge = CvBridge()
            self.saved = 0
            self.previous = None
            self.last_save = 0.0
            self.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)

        def on_image(self, message):
            now = time.monotonic()
            if now - self.last_save < args.interval:
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            except Exception as error:
                self.get_logger().error(f"Не удалось преобразовать кадр: {error}")
                return
            preview = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (64, 48))
            if self.previous is not None:
                change = float(np.mean(cv2.absdiff(preview, self.previous)))
                if change < args.min_change:
                    return
            filename = output / f"frame_{datetime.now():%Y%m%d_%H%M%S_%f}_{self.saved:03d}.jpg"
            if not cv2.imwrite(
                str(filename), frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
            ):
                self.get_logger().error(f"Не удалось записать {filename}")
                return
            self.previous = preview
            self.last_save = now
            self.saved += 1
            print(f"[{self.saved}/{args.count}] {filename}", flush=True)

    rclpy.init()
    node = Collector()
    started = time.monotonic()
    print(f"Камера: {args.topic}")
    print("Плавно двигайте RMC1 или манипулятор, чтобы ракурсы менялись.")
    try:
        while rclpy.ok() and node.saved < args.count:
            if time.monotonic() - started >= args.timeout:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        saved = node.saved
        node.destroy_node()
        rclpy.shutdown()
    print(f"Готово: сохранено {saved} кадров в {output}")
    if saved == 0:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
