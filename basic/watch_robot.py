#!/usr/bin/env python3
"""Показывает одометрию и минимальную дистанцию перед ровером."""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=("RMC1", "RMC2"), required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Node("mvch_basic_watch")
    state = {"pose": None, "front": None}

    def on_odom(message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        state["pose"] = (position.x, position.y, math.atan2(siny, cosy))

    def on_scan(message):
        front = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) <= math.radians(20) and math.isfinite(distance):
                if message.range_min <= distance <= message.range_max:
                    front.append(distance)
        state["front"] = min(front, default=math.inf)

    node.create_subscription(
        Odometry, f"/{args.robot}/odometry", on_odom, qos_profile_sensor_data
    )
    node.create_subscription(
        LaserScan, f"/{args.robot}/scan_front", on_scan, qos_profile_sensor_data
    )

    print(f"Слушаю {args.robot}. Ctrl+C — выход.")
    last_print = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last_print >= 0.5:
                pose = state["pose"]
                front = state["front"]
                pose_text = "нет odometry" if pose is None else (
                    f"x={pose[0]:.2f}, y={pose[1]:.2f}, yaw={math.degrees(pose[2]):.1f}°"
                )
                front_text = "нет scan_front" if front is None else f"{front:.2f} м"
                print(f"{pose_text}; спереди: {front_text}")
                last_print = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

