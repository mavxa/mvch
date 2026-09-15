#!/usr/bin/env python3
from datetime import datetime
from pathlib import Path
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ACTION = "/RMC1/arm95/gripper_controller/gripper_cmd"
TOPIC = "/RMC1/arm95/gripper_trajectory_controller/joint_trajectory"
HERE = Path(__file__).resolve().parent


class Gripper(Node):
    def __init__(self):
        super().__init__("simple_gripper")
        self.client = ActionClient(self, GripperCommand, ACTION)
        self.publisher = self.create_publisher(JointTrajectory, TOPIC, 10)

    def set_state(self, opened):
        state = "open" if opened else "closed"
        if self.client.wait_for_server(timeout_sec=2.0):
            goal = GripperCommand.Goal()
            goal.command.position = 0.04 if opened else 0.0
            goal.command.max_effort = 20.0
            future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("Контроллер отклонил команду")
            result = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result)
        else:
            message = JointTrajectory()
            message.joint_names = ["left_joint", "right_joint"]
            point = JointTrajectoryPoint()
            value = 0.04 if opened else 0.0213
            point.positions = [value, value]
            point.time_from_start = Duration(sec=1)
            message.points = [point]
            for _ in range(5):
                self.publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.1)

        line = f"{datetime.now().isoformat(timespec='seconds')} | gripper={state}"
        print(line)
        with (HERE / "logs/simple_gripper.log").open("a", encoding="utf-8") as log:
            log.write(line + "\n")
        time.sleep(1)


def main():
    rclpy.init()
    node = Gripper()
    try:
        node.set_state(True)
        input("Захват открыт. Enter — закрыть: ")
        node.set_state(False)
        input("Захват закрыт. Enter — снова открыть: ")
        node.set_state(True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
