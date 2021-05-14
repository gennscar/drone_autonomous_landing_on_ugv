#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from gazebo_msgs.msg import UwbSensor
from geometry_msgs.msg import PointStamped

import functions


class UwbPositioning(Node):
    """
    Node to estimate the position using only the UWB

    Params:
      sensor_id (str): The anchor ID of the UWB sensor that need to be tracked
        is the <anchor_id> param provided @ the UWB gazebo plugin

      method (str): Can be "LS" or "GN":
        "LS" is the Least-Square method
        "GN" is for the Gauss-Newton method

      iterations (int): Numbers of iterations, valid only with the GN method
    """

    def __init__(self):
        super().__init__("uwb_positioning")

        self.anchors_ = {}
        self.sensor_est_pos_ = [0.01, 0.01, 0.01]  # To be randomized

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

        # Namespace check
        if(self.get_namespace() == '/'):
            self.get_logger().error("uwb_positioning need a namespace")
            self.destroy_node()

        # Setting up sensors subscriber for the UWB plugin
        self.sensor_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor_" + self.sensor_id_, self.callback_sensor_subscriber, 10)

        # Setting up a publishers to send the estimated position
        self.estimator_topic_name_ = self.get_namespace() + "/estimated_pos"
        self.position_mse_publisher_ = self.create_publisher(
            PointStamped, self.estimator_topic_name_, 10)

        self.get_logger().info(f"""
                                Node has started:
                                  Sensor ID:  {self.sensor_id_}
                                  Method:     {self.method_}
                                  Iterations  {self.iterations_}
                                """)

    def callback_sensor_subscriber(self, msg):
        """
        Receive a pseudorange from the UWB sensor

        Args:
            msg (UwbSensor): The message received by the plugin
        """

        # Saving the message in a dict
        self.anchors_[msg.anchor_id] = msg

        # Only if trilateration is possible
        if len(self.anchors_) > 3:
            if(self.method_ == "LS"):
                self.sensor_est_pos_ = functions.ls_trilateration(
                    self.anchors_)

            if(self.method_ == "GN"):
                # Perform N iterations of GN
                for _ in range(self.iterations_):
                    self.sensor_est_pos_ = functions.gauss_newton_trilateration(
                        self.sensor_est_pos_, self.anchors_)

            # Sending the estimated position and the name of the node that generated it
            msg = PointStamped()
            msg.header.frame_id = self.estimator_topic_name_
            msg.header.stamp = self.get_clock().now().to_msg()
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
