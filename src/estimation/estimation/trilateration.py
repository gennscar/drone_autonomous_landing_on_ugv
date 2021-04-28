#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from gazebo_msgs.msg import UwbSensor
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import functions


class Trilateration(Node):
    """Node to estimate the position using trilateration methods"""

    def __init__(self):
        super().__init__("trilateration")

        self.anchors = {}
        self.sensor_true_pos = []
        self.sensor_est_pos = [0.01, 0.01, 0.01]

        # Parameters declaration
        self.sensor_id_ = self.declare_parameter("sensor_id", "0")
        self.method_ = self.declare_parameter("method", "LS")

        # Retrieve parameter values
        self.sensor_id_ = self.get_parameter(
            "sensor_id").get_parameter_value().string_value
        self.method_ = self.get_parameter(
            "method").get_parameter_value().string_value

        # Setting up sensors subscribers
        self.sensor_subscriber = self.create_subscription(
            UwbSensor, "/uwb_sensor_" + self.sensor_id_, self.callback_sensor_subscriber, 10)
        self.sensor_true_subscriber = self.create_subscription(
            Odometry, "/uwb_sensor_" + self.sensor_id_ + "_true/odom", self.callback_sensor_true_subscriber, 10)

        # Setting up a publishers to record the estimation error
        self.position_mse_publisher = self.create_publisher(
            Float64, "/position_error_" + self.method_ + "_" + self.sensor_id_, 10)

        self.get_logger().info("trilateration has started")

    def callback_sensor_subscriber(self, msg):
        self.anchors[msg.anchor_id] = msg

        if len(self.anchors) > 3 and self.sensor_true_pos != []:
            if(self.method_ == "LS"):
                self.sensor_est_pos = functions.ls_trilateration(
                    self.anchors)

            if(self.method_ == "GN"):
                self.sensor_est_pos = functions.gauss_newton_trilateration(
                    self.sensor_est_pos, self.anchors)

            est_error = np.linalg.norm(
                self.sensor_est_pos - self.sensor_true_pos, ord=2)

            msg = Float64()
            msg.data = est_error
            self.position_mse_publisher.publish(msg)

    def callback_sensor_true_subscriber(self, msg):
        self.sensor_true_pos = [msg.pose.pose.position.x,
                                msg.pose.pose.position.y,
                                msg.pose.pose.position.z]


def main(args=None):
    rclpy.init(args=args)
    node = Trilateration()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
