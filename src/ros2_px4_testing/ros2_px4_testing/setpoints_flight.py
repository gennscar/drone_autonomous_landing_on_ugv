#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys

from ros2_px4_interfaces.srv import ControlMode

setpoints = [
    {'mode': 'Nope', 't': 10},
    {'mode': 'setpoint', 'x': 0.,   'y': 0.,    'z': 3.0, 't': 10},
    {'mode': 'setpoint', 'x': 25.,  'y': 0.,    'z': 3.0, 't': 15},
    {'mode': 'setpoint', 'x': 25.,  'y': 5.,    'z': 3.0, 't': 5},
    {'mode': 'setpoint', 'x': 20.,  'y': 5.,    'z': 3.0, 't': 5},
    {'mode': 'setpoint', 'x': 20.,  'y': -20.,  'z': 3.0, 't': 10},
    {'mode': 'land', 't': 5}
]


class SetpointsFlight(Node):
    """@todo"""

    def __init__(self):
        super().__init__("setpoints_flight")

        # Counter to sync setpoints sequence
        self.index_ = 0
        self.counter_ = setpoints[0]['t']

        # Client to DroneController service
        self.client_ = self.create_client(
            ControlMode, 'ControlMode_Service')
        self.request_ = ControlMode.Request()

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
                sys.exit()

            # Setting up the counter
            self.counter_ = setpoints[self.index_]['t']

            # Sending request
            self.request_.control_mode = setpoints[self.index_]['mode']
            if self.request_.control_mode == 'setpoint':
                self.request_.x = setpoints[self.index_]['x']
                self.request_.y = setpoints[self.index_]['y']
                self.request_.z = setpoints[self.index_]['z']
            self.client_.call_async(self.request_)

            # Logging
            self.get_logger().info(f"""Sending request:
                                   index: {self.index_}
                                   mode: {self.request_.control_mode}
                                   """)

        # Decreasing counter for current setpoint
        self.counter_ -= 1


def main(args=None):
    rclpy.init(args=args)
    node = SetpointsFlight()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
