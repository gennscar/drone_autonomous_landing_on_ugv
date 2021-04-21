#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from gazebo_msgs.msg import UwbSensor

from common_modules.trilateration import ls_trilateration


class UwbVehicle(Node):
    def __init__(self):
        super().__init__("uwb_vehicle")

        self.drone_position_subscriber = self.create_subscription(
            VehicleLocalPosition, "VehicleLocalPosition_PubSubTopic", self.callback_drone_position, 10)
        self.uwb_sensor_subscriber = self.create_subscription(
            UwbSensor, "uwb_sensor_0", self.callback_uwb_sensor, 10)

        self.get_logger().info("uwb_vehicle has started")
        self.timer = self.create_timer(1, self.timer_callback)

        self.anchors = {}

    def callback_drone_position(self, msg):
        self.drone_position = [msg.x, msg.y, msg.z]

    def callback_uwb_sensor(self, msg):
        self.anchors[msg.anchor_id] = msg

    def timer_callback(self):
        i = 0
        if len(self.anchors) > 3:
            y = ls_trilateration(self.anchors)
            print(y[1:])


def main(args=None):
    rclpy.init(args=args)
    node = UwbVehicle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
