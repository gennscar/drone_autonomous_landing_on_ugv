#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from ros2_px4_interfaces.srv import ControlMode

setpoints = [
    {'x': 1.0, 'y': 1.0, 'z': 1.0, 't': 10},
    {'x': 1.0, 'y': 1.0, 'z': 10.0, 't': 10},
    {'x': 49.0, 'y': 1.0, 'z': 10.0, 't': 10},
    {'x': 49.0, 'y': 49.0, 'z': 10.0, 't': 10},
    {'x': 1.0, 'y': 49.0, 'z': 10.0, 't': 10},
    {'x': 1.0, 'y': 1.0, 'z': 10.0, 't': 10}
]


class SetpointsFlight(Node):
    """@todo"""

    def __init__(self):
        super().__init__("setpoints_flight")

        # Counter to sync setpoints sequence
        self.index_ = 0
        self.counter_ = setpoints[0]['t']

        # Client to DroneController service
        self.client_ = self.create_client(ControlMode, '/control_mode')
        self.request_ = ControlMode.Request()
        self.request_.control_mode = "setpoint_mode"

        # Timer to send the setpoints request
        self.timer_ = self.create_timer(1, self.timer_callback)

        self.get_logger().info(f"""Node has started""")

    def timer_callback(self):
        """Timer to send the setpoints request"""

        # Passing to the next setpoint if the last is finished
        if (self.counter_ == 0):
            self.index_ += 1

            # Checking if there are more checkpoints
            if(self.index_ >= len(setpoints)):
                self.get_logger().info(f"""No more setpoints to reach !!!""")
                self.destroy_node()
                return

            # Setting up the counter
            self.counter_ = setpoints[self.index_]['t']

        # Sending request
        self.request_.x = setpoints[self.index_]['x']
        self.request_.y = setpoints[self.index_]['y']
        self.request_.z = setpoints[self.index_]['z']
        self.client_.call_async(self.request_)

        # Decreasing counter for current setpoint
        self.counter_ -= 1

        # Logging
        self.get_logger().info(f"""Sending request...
                               index: {self.index_}
                               x:     {self.request_.x}
                               y:     {self.request_.y}
                               z:     {self.request_.z}
                               t:     {self.counter_}
                               """)


def main(args=None):
    rclpy.init(args=args)
    node = SetpointsFlight()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
