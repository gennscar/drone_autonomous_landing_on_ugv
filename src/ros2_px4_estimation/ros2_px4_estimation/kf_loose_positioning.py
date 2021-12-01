#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from nav_msgs.msg import Odometry


class KfLoosePositioning(Node):
    """
    EF
    """

    def __init__(self):
        super().__init__("kf_loose_positioning")

        self.declare_parameters("", [
            ("delta_t", 0.02),
            ("q", 0.1),
            ("r_uwb", 0.0025),
            ("r_laser", 0.01)
        ])

        self.params_ = {
            x.name: x.value for x in self.get_parameters(
                ["delta_t", "q", "r_uwb", "r_laser"]
            )
        }

        # Kalman Filter
        self.kalman_filter_ = KalmanFilter(dim_x=9, dim_z=9)

        # State transition matrix
        f = np.array([
            [1., self.params_["delta_t"], 0.5*self.params_["delta_t"]**2.],
            [0., 1.,         self.params_["delta_t"]],
            [0., 0.,         1.]
        ])
        self.kalman_filter_.F = scipy.linalg.block_diag(*[f]*3)

        # Process noise
        self.kalman_filter_.Q = Q_discrete_white_noise(
            dim=3, dt=self.params_["delta_t"], var=self.params_["q"], block_size=3)

        # Covariance matrix
        self.kalman_filter_.P *= 1.

        # Setting up sensors subscribers
        self.sensor_subscriber_ = self.create_subscription(
            Odometry, "UwbPositioning/Odometry", self.callback_uwb_subscriber, 10)

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            Odometry, "~/Odometry", 10)

        # Prediction timer
        self.timer = self.create_timer(
            self.params_["delta_t"], self.predict_callback)

        self.get_logger().info("Node has started")

    def callback_uwb_subscriber(self, msg):
        # Storing current estimate in a np.array
        z = np.array([
            msg.pose.pose.position.x, 0., 0.,
            msg.pose.pose.position.y, 0., 0.,
            msg.pose.pose.position.z, 0., 0.,
        ])

        h = np.array([
            [1., 0., 0.],
            [0., 0., 0.],
            [0., 0., 0.]
        ])
        H = scipy.linalg.block_diag(*[h]*3)

        # Filter update
        self.kalman_filter_.update(z, self.params_["r_uwb"], H)

    def predict_callback(self):
        # Sending the estimated position
        msg = Odometry()
        msg.header.frame_id = "/KfPositioning"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.kalman_filter_.x[0][0]
        msg.pose.pose.position.y = self.kalman_filter_.x[3][0]
        msg.pose.pose.position.z = self.kalman_filter_.x[6][0]

        msg.pose.covariance[0] = self.kalman_filter_.P[0][0]
        msg.pose.covariance[1] = self.kalman_filter_.P[3][3]
        msg.pose.covariance[2] = self.kalman_filter_.P[6][6]

        self.est_pos_publisher_.publish(msg)

        self.kalman_filter_.predict()


def main(args=None):
    rclpy.init(args=args)
    node = KfLoosePositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
