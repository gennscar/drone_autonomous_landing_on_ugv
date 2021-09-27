#!/usr/bin/env python3


import numpy as np
from numpy.core.numeric import NaN
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration

from ros2_px4_interfaces.msg import UwbSensor
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleVisualOdometry, Timesync

import ros2_px4_functions


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
        self.sensor_est_pos_ = [0.01, 0.01, 0.01]  # @todo Need to be random

        # Parameters declaration
        self.sensor_id_ = self.declare_parameter("sensor_id", "Iris")
        self.method_ = self.declare_parameter("method", "LS")
        self.iterations_ = self.declare_parameter("iterations", 1)
        self.allowed_delay_ns = self.declare_parameter("allowed_delay_ns", 1e8)
        self.max_range = self.declare_parameter("max_range", 30.0)

        # Retrieve parameter values
        self.sensor_id_ = self.get_parameter(
            "sensor_id").get_parameter_value().string_value
        self.method_ = self.get_parameter(
            "method").get_parameter_value().string_value
        self.iterations_ = self.get_parameter(
            "iterations").get_parameter_value().integer_value
        self.allowed_delay_ns = self.get_parameter(
            "allowed_delay_ns").get_parameter_value().double_value
        self.max_range = self.get_parameter(
            "max_range").get_parameter_value().double_value

        # Namespace check
        if(self.get_namespace() == '/'):
            self.get_logger().error("uwb_positioning need a namespace")
            self.destroy_node()

        self.timestamp_ = 0

        # Setting up sensors subscriber for the UWB plugin
        self.sensor_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor/" + self.sensor_id_, self.callback_sensor_subscriber, 10)

        # Setting up a publishers to send the estimated position
        self.estimator_topic_name_ = self.get_namespace() + "/estimated_pos"
        self.position_mse_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, self.estimator_topic_name_, 10
        )

        self.get_logger().info(f"""Node has started:
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
        self.anchors_[msg.anchor_pose.header.frame_id] = msg

        # Extract anchor positions and ranges
        i = 0
        anchor_pos = np.empty((len(self.anchors_), 3))
        ranges = np.empty(len(self.anchors_))

        for _, data in self.anchors_.items():
            delta = Time.from_msg(msg.anchor_pose.header.stamp) - \
                Time.from_msg(data.anchor_pose.header.stamp)

            if delta < Duration(nanoseconds=self.allowed_delay_ns) and data.range > 0.0 and data.range <= self.max_range:
                anchor_pos[i, :] = np.array(
                    [data.anchor_pose.pose.position.x, data.anchor_pose.pose.position.y, data.anchor_pose.pose.position.z])
                ranges[i] = data.range
                i = i+1

        N = i
        anchor_pos = anchor_pos[0:N, :]
        ranges = ranges[0:N]

        # Only if trilateration is possible
        if N > 2:
            # Perform Least-Square
            if(self.method_ == "LS"):
                self.sensor_est_pos_ = ros2_px4_functions.ls_trilateration(
                    anchor_pos, ranges, N)

            # Perform Gauss-Newton
            if(self.method_ == "GN"):
                # Perform N iterations of GN
                for _ in range(self.iterations_):
                    self.sensor_est_pos_ = ros2_px4_functions.gauss_newton_trilateration(
                        self.sensor_est_pos_, anchor_pos, ranges)

            # Sending the estimated position and the name of the node that generated it
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.estimator_topic_name_
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = self.sensor_est_pos_[0]
            msg.pose.pose.position.y = self.sensor_est_pos_[1]
            msg.pose.pose.position.z = self.sensor_est_pos_[2]

            msg.pose.covariance[0] = 6.25e-4
            msg.pose.covariance[1] = 0.0
            msg.pose.covariance[2] = 0.0
            msg.pose.covariance[6] = 0.0
            msg.pose.covariance[7] = 6.25e-4
            msg.pose.covariance[8] = 0.0
            msg.pose.covariance[12] = 0.0
            msg.pose.covariance[13] = 0.0
            msg.pose.covariance[14] = 1e0

            self.position_mse_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
