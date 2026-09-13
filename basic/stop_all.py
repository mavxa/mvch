#!/usr/bin/env python3
"""Публикует нулевую скорость обоим роверам."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("mvch_basic_stop_all")
    publishers = [
        node.create_publisher(Twist, "/RMC1/cmd_vel", 10),
        node.create_publisher(Twist, "/RMC2/cmd_vel", 10),
    ]
    stop = Twist()
    for _ in range(10):
        for publisher in publishers:
            publisher.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.02)
    print("Нулевая скорость отправлена RMC1 и RMC2")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

