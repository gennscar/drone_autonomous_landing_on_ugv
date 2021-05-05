#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from gazebo_msgs.msg import UwbSensor
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import functions

# seed the pseudorandom number generator
from random import seed
from random import random
# seed random number generator
seed(1)

STD_TRILATERATION = False


class UwbPosEstimation(Node):
    def __init__(self):
        super().__init__("uwb_pos_estimation")

        self.anchors = {}
        self.sensor_true_pos = []
        self.sensor_est_pos = [random(), random(), random()]

        self.sensor_subscriber = self.create_subscription(
            UwbSensor, "/uwb_sensor_0", self.callback_sensor_subscriber, 10)
        self.sensor_true_subscriber = self.create_subscription(
            Odometry, "/uwb_sensor_true/odom", self.callback_sensor_true_subscriber, 10)

        self.position_mse_publisher = self.create_publisher(
            Float64, "/position_mse", 10)

        self.get_logger().info("uwb_pos_estimation has started")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_sensor_subscriber(self, msg):
        self.anchors[msg.anchor_id] = msg

    def callback_sensor_true_subscriber(self, msg):
        self.sensor_true_pos = [msg.pose.pose.position.x,
                                msg.pose.pose.position.y,
                                msg.pose.pose.position.z]

    def timer_callback(self):
        if len(self.anchors) > 3:
            if STD_TRILATERATION == True:
                self.sensor_est_pos = functions.ls_trilateration(self.anchors)
            else:
                self.sensor_est_pos = functions.gauss_newton_trilateration(
                    self.sensor_est_pos, self.anchors)

            self.err = np.linalg.norm(
                self.sensor_est_pos - self.sensor_true_pos, ord=2)

            msg = Float64()
            msg.data = self.err
            self.position_mse_publisher.publish(msg)

            self.get_logger().info(f"""
            Computed pos: {self.sensor_est_pos}
            True pos: {self.sensor_true_pos}
            Error: {self.err}""")


def main(args=None):
    rclpy.init(args=args)
    node = UwbPosEstimation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
