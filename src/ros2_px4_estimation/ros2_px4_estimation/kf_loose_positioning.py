#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleLocalPosition


class KfLoosePositioning(Node):
    """
    EKF
    """

    def __init__(self):
        super().__init__("kf_loose_positioning")

        dT = 0.01

        # Kalman Filter
        self.kalman_filter_ = KalmanFilter(dim_x=9, dim_z=9)

        # State transition matrix
        f = np.array([
            [1., dT, 0.5*dT**2.],
            [0., 1.,         dT],
            [0., 0.,         1.]
        ])
        self.kalman_filter_.F = scipy.linalg.block_diag(*[f]*3)

        # Observation matrix
        self.kalman_filter_.H = np.eye(9)

        # Process noise
        self.kalman_filter_.Q = Q_discrete_white_noise(
            dim=3, dt=dT, var=1e-2, block_size=3)

        # Covariance matrix
        self.kalman_filter_.P *= 1000.

        # Setting up sensors subscribers
        self.sensor_subscriber_ = self.create_subscription(
            PointStamped, "/LS_uwb_estimator/estimated_pos", self.callback_uwb_subscriber, 10)
        self.sensor_subscriber_ = self.create_subscription(
            VehicleLocalPosition, "/VehicleLocalPosition_PubSubTopic", self.callback_px4_subscriber, 10)

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PointStamped, self.get_namespace() + "/estimated_pos", 10)

        # Prediction timer
        self.timer = self.create_timer(dT, self.predict_callback)

        self.get_logger().info("Node has started")

    def callback_uwb_subscriber(self, msg):
        # Storing current estimate in a np.array
        z = np.array([
            msg.point.x, 0., 0.,
            msg.point.y, 0., 0.,
            msg.point.z, 0., 0.,
        ])

        R = 0.01

        h = np.array([
            [1., 0., 0.],
            [0., 0., 0.],
            [0., 0., 0.]
        ])
        H = scipy.linalg.block_diag(*[h]*3)

        # Filter update
        self.kalman_filter_.update(z, R, H)

    def callback_px4_subscriber(self, msg):
        # Storing current estimate in a np.array
        z = np.array([
            msg.y + 1.0,  msg.vy,  msg.ay,
            msg.x + 1.0,  msg.vx,  msg.ax,
            -msg.z,      -msg.vz, -msg.az
        ])
        R = 1

        # Filter update
        self.kalman_filter_.update(z, R)

    def predict_callback(self):
        # Sending the estimated position
        msg = PointStamped()
        msg.header.frame_id = self.get_namespace() + "/estimated_pos"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = self.kalman_filter_.x[0][0]
        msg.point.y = self.kalman_filter_.x[3][0]
        msg.point.z = self.kalman_filter_.x[6][0]
        self.est_pos_publisher_.publish(msg)

        self.kalman_filter_.predict()


def main(args=None):
    rclpy.init(args=args)
    node = KfLoosePositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
