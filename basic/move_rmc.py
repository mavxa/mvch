#!/usr/bin/env python3
"""Подаёт ограниченную по времени ROS-команду скорости и затем останавливает RMC."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=("RMC1", "RMC2"), required=True)
    parser.add_argument("--x", type=float, default=0.0, help="Скорость вперёд, м/с")
    parser.add_argument("--y", type=float, default=0.0, help="Боковая скорость RMC1, м/с")
    parser.add_argument("--yaw", type=float, default=0.0, help="Угловая скорость, рад/с")
    parser.add_argument("--seconds", type=float, default=1.0)
    parser.add_argument("--stop-distance", type=float, default=0.55, help="Стоп по lidar, м")
    parser.add_argument("--no-lidar", action="store_true", help="Только стенд без препятствий")
    args = parser.parse_args()

    if args.seconds <= 0:
        parser.error("--seconds должен быть больше нуля")
    if args.robot == "RMC2" and args.y != 0:
        parser.error("RMC2 не поддерживает боковое движение: используйте --y 0")

    rclpy.init()
    node = Node("mvch_basic_move")
    publisher = node.create_publisher(Twist, f"/{args.robot}/cmd_vel", 10)
    front_distance = [None]

    def on_scan(message):
        if not message.ranges or message.angle_increment == 0:
            return
        front = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) <= 0.44 and math.isfinite(distance):
                if message.range_min <= distance <= message.range_max:
                    front.append(distance)
        front_distance[0] = min(front, default=math.inf)

    if not args.no_lidar:
        node.create_subscription(
            LaserScan,
            f"/{args.robot}/scan_front",
            on_scan,
            qos_profile_sensor_data,
        )
    command = Twist()
    command.linear.x = args.x
    command.linear.y = args.y
    command.angular.z = args.yaw

    print(f"{args.robot}: cmd_vel x={args.x}, y={args.y}, yaw={args.yaw}")
    if not args.no_lidar:
        print(f"Жду /{args.robot}/scan_front...")
        deadline = time.monotonic() + 3.0
        while front_distance[0] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if front_distance[0] is None:
            node.destroy_node()
            rclpy.shutdown()
            raise SystemExit("Нет scan_front: движение запрещено. Для стенда: --no-lidar")
    started = time.monotonic()
    exit_code = 0
    try:
        while time.monotonic() - started < args.seconds:
            if front_distance[0] is not None and front_distance[0] < args.stop_distance:
                raise RuntimeError(
                    f"Препятствие {front_distance[0]:.2f} м — аварийный стоп"
                )
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
    except RuntimeError as error:
        exit_code = 1
        print(f"STOP: {error}")
    finally:
        stop = Twist()
        for _ in range(5):
            publisher.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
