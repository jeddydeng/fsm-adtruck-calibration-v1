#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def main() -> None:
    parser = argparse.ArgumentParser(description="Save one telephoto ROS2 compressed image")
    parser.add_argument("--topic", default=
                        "/sensing/camera/traffic_light_front/image_raw/compressed")
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=5.0)
    arguments = parser.parse_args()

    rclpy.init()
    node = Node("telephoto_capture_once")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    received = []

    def callback(message: CompressedImage) -> None:
        received.append(bytes(message.data))

    subscription = node.create_subscription(CompressedImage, arguments.topic, callback, qos)
    deadline = time.monotonic() + arguments.timeout
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    if not received:
        raise SystemExit("timed out waiting for telephoto image")
    output = Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(received[-1])
    print(f"saved {len(received[-1])} bytes to {output}")


if __name__ == "__main__":
    main()
