#!/usr/bin/env python3
"""Поднимает или опускает лифт RMC2 через штатный ROS-топик."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("state", choices=("up", "down"))
    parser.add_argument("--wait", type=float, default=6.0, help="Сколько ждать статус, с")
    args = parser.parse_args()

    rclpy.init()
    node = Node("mvch_basic_lift")
    exit_message = None
    try:
        publisher = node.create_publisher(Float64, "/RMC2/lift", 10)
        statuses = []
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            String,
            "/RMC2/lift_status",
            lambda msg: statuses.append(msg.data),
            status_qos,
        )

        height = 0.05 if args.state == "up" else 0.0
        expected = "raised" if args.state == "up" else "lowered"
        print(f"RMC2 lift: {args.state}, команда {height}")
        for _ in range(5):
            publisher.publish(Float64(data=height))
            rclpy.spin_once(node, timeout_sec=0.1)

        deadline = time.monotonic() + args.wait
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if statuses and statuses[-1].strip().lower() == expected:
                break

        if statuses and statuses[-1].strip().lower() == expected:
            print(f"Статус: {statuses[-1]}")
        else:
            current = statuses[-1] if statuses else "нет данных"
            exit_message = (
                f"Лифт не подтвердил {expected}; текущий статус: {current}"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if exit_message:
        raise SystemExit(exit_message)


if __name__ == "__main__":
    main()
