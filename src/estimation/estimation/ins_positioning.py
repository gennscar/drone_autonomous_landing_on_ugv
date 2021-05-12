#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node


class InsPositioning(Node):
    """Node to estimate the position using INS"""

    def __init__(self):
        super().__init__("ins_positioning")

        # Setting up sensors subscribers
        self.sensor_subscriber = self.create_subscription(
            UwbSensor, "", 10)

        self.get_logger().info("ins_positioning has started")


def main(args=None):
    rclpy.init(args=args)
    node = InsPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
