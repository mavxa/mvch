#!/usr/bin/env python3
import time
from pathlib import Path

import cv2
import rclpy
from control_msgs.action import GripperCommand
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
CAMERA = "/RMC1/arm95/camera_gripper/image_color"
GRIPPER = "/RMC1/arm95/gripper_controller/gripper_cmd"


class Demo(Node):
    def __init__(self):
        super().__init__("simple_module_b")
        self.model = YOLO(str(HERE / "models/latest.pt"))
        self.bridge = CvBridge()
        self.gripper = ActionClient(self, GripperCommand, GRIPPER)
        self.gripper.wait_for_server()
        self.create_subscription(Image, CAMERA, self.on_image, qos_profile_sensor_data)
        self.started = None
        self.opened = False
        self.closed = False

    def move_gripper(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 20.0
        self.gripper.send_goal_async(goal)

    def on_image(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
        result = self.model(frame, conf=0.25, verbose=False)[0]

        if self.started is None:
            self.started = time.monotonic()
        elapsed = time.monotonic() - self.started

        if elapsed >= 5 and not self.opened:
            self.move_gripper(0.04)
            self.opened = True
            print("Захват открыт")
        elif elapsed >= 7 and not self.closed:
            self.move_gripper(0.0)
            self.closed = True
            print("Захват закрыт")

        cv2.imshow("YOLO", result.plot())
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            rclpy.shutdown()


def main():
    input("Предмет (Кисть, Нож или Резаки): ")
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
