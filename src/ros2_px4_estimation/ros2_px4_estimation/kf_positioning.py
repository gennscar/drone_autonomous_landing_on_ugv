#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from geometry_msgs.msg import PointStamped


class UwbEfk(Node):
    """
    EKF
    """

    def __init__(self):
        super().__init__("uwb_ekf")

        dT = 1./5.

        # Kalman Filter 2 states (z and vz), 1 measurement (z)
        self.kalman_filter_ = KalmanFilter(dim_x=6, dim_z=3)

        # Initial state
        self.kalman_filter_.x = np.array([0., 0., 0., 0., 0., 0.])

        # State transition matrix x y z dx dy dz
        self.kalman_filter_.F = np.array([[1., dT], [0., 1.]])

        # Observation matrix
        self.kalman_filter_.H = np.array([[1., 0.]])

        # Covariance matrix
        self.kalman_filter_.P *= 1000.

        # Measurement noise
        self.kalman_filter_.R = 0.01

        # Process noise
        self.kalman_filter_.Q = Q_discrete_white_noise(
            dim=2, dt=dT, var=0.13, block_size=3)

        # Setting up sensors subscribers
        self.sensor_subscriber_ = self.create_subscription(
            PointStamped, "/LS_uwb_estimator/estimated_pos", self.callback_sensor_subscriber, 10)

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PointStamped, self.get_namespace() + "/estimated_pos", 10)

        self.get_logger().info("Node has started")

    def callback_sensor_subscriber(self, msg):
        measurement = np.array([msg.point.x,
                                msg.point.y,
                                msg.point.z])

        self.kalman_filter_.predict()
        self.kalman_filter_.update(measurement[2])

        # Sending the estimated position
        msg = PointStamped()
        msg.header.frame_id = self.get_namespace() + "/estimated_pos"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = 0.0
        msg.point.y = 0.0
        msg.point.z = self.kalman_filter_.x[0]

        self.est_pos_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbEfk()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
