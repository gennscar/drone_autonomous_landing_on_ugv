#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from gazebo_msgs.msg import UwbSensor
from geometry_msgs.msg import PointStamped

import functions


class UwbPositioning(Node):
    """Node to estimate the position using uwb_positioning methods"""

    def __init__(self):
        super().__init__("uwb_positioning")

        self.anchors_ = {}
        self.sensor_est_pos_ = [0.01, 0.01, 0.01]

        # Parameters declaration
        self.sensor_id_ = self.declare_parameter("sensor_id", "0")
        self.method_ = self.declare_parameter("method", "LS")
        self.iterations_ = self.declare_parameter("iterations", 1)

        # Retrieve parameter values
        self.sensor_id_ = self.get_parameter(
            "sensor_id").get_parameter_value().string_value
        self.method_ = self.get_parameter(
            "method").get_parameter_value().string_value
        self.iterations_ = self.get_parameter(
            "iterations").get_parameter_value().integer_value

        # Setting up sensors subscribers
        self.sensor_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor_" + self.sensor_id_, self.callback_sensor_subscriber, 10)

        # Setting up a publishers to record the estimation error
        self.estimator_topic_name_ = self.get_namespace() + "/estimated_pos"
        self.position_mse_publisher_ = self.create_publisher(
            PointStamped, self.estimator_topic_name_, 10)

        self.get_logger().info("uwb_positioning has started")

    def callback_sensor_subscriber(self, msg):
        self.anchors_[msg.anchor_id] = msg

        if len(self.anchors_) > 3:
            if(self.method_ == "LS"):
                self.sensor_est_pos_ = functions.ls_trilateration(
                    self.anchors_)

            if(self.method_ == "GN"):
                for _ in range(self.iterations_):
                    self.sensor_est_pos_ = functions.gauss_newton_trilateration(
                        self.sensor_est_pos_, self.anchors_)

            msg = PointStamped()
            msg.header.frame_id = self.estimator_topic_name_
            msg.point.x = self.sensor_est_pos_[0]
            msg.point.y = self.sensor_est_pos_[1]
            msg.point.z = self.sensor_est_pos_[2]

            self.position_mse_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
