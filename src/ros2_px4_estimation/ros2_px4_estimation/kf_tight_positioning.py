#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import UwbSensor


class UwbEfk(Node):
    """
    EKF
    """

    def __init__(self):
        super().__init__("uwb_ekf")

        dT = 0.2
        self.anchors_ = []

        # Kalman Filter
        self.kalman_filter_ = KalmanFilter(dim_x=6, dim_z=4)

        # Initial state x dx y dy z dz
        self.kalman_filter_.x = np.array([0.1, 0., 0.1, 0., 0.1, 0.])

        # State transition matrix
        self.kalman_filter_.F = np.array([
            [1., dT, 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [0., 0., 1., dT, 0., 0.],
            [0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 1., dT],
            [0., 0., 0., 0., 0., 1.]
        ])

        # Covariance matrix
        self.kalman_filter_.P *= 1000.

        # Measurement noise
        self.kalman_filter_.R = 0.01

        # Process noise
        self.kalman_filter_.Q = Q_discrete_white_noise(
            dim=2, dt=dT, var=1e-4, block_size=3)

        # Setting up sensors subscribers
        self.sensor_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor_0", self.callback_sensor_subscriber, 10)

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PointStamped, self.get_namespace() + "/estimated_pos", 10)

        self.get_logger().info("Node has started")

    def callback_sensor_subscriber(self, msg):
        # Saving the message in a dict
        anchor_data = {}
        anchor_data['timestamp'] = msg.timestamp
        anchor_data['anchor_pos'] = np.array([
            msg.anchor_pos.x,
            msg.anchor_pos.y,
            msg.anchor_pos.z
        ])
        anchor_data['range'] = msg.range
        self.anchors_.append(anchor_data)

        # Removing old anchors
        tmp_anchors = [
            a for a in self.anchors_ if msg.timestamp - a['timestamp'] < 0.01]
        self.anchors_ = tmp_anchors

        # Initiating update only if 4 ranges are avaiables
        if len(self.anchors_) < 4:
            return

        # Storing current estimate in a np.array
        p = np.array([self.kalman_filter_.x[0],
                      self.kalman_filter_.x[2],
                      self.kalman_filter_.x[4]])

        # Calculating the ranges with the estimated position
        est_ranges = [np.linalg.norm(p - a['anchor_pos'])
                      for a in self.anchors_]

        self.get_logger().info(f"""
                               x          {self.kalman_filter_.x}
                               p          {p}
                               est_ranges {est_ranges}
                               anchors_   {self.anchors_}
                               """)

        # Deriving H
        self.kalman_filter_.H = np.array([
            [
                (p[0] - self.anchors_[0]['anchor_pos'][0]) / est_ranges[0], 0.,
                (p[1] - self.anchors_[0]['anchor_pos'][1]) / est_ranges[0], 0.,
                (p[2] - self.anchors_[0]['anchor_pos'][2]) / est_ranges[0], 0.
            ],
            [
                (p[0] - self.anchors_[1]['anchor_pos'][0]) / est_ranges[1], 0.,
                (p[1] - self.anchors_[1]['anchor_pos'][1]) / est_ranges[1], 0.,
                (p[2] - self.anchors_[1]['anchor_pos'][2]) / est_ranges[1], 0.
            ],
            [
                (p[0] - self.anchors_[2]['anchor_pos'][0]) / est_ranges[2], 0.,
                (p[1] - self.anchors_[2]['anchor_pos'][1]) / est_ranges[2], 0.,
                (p[2] - self.anchors_[2]['anchor_pos'][2]) / est_ranges[2], 0.
            ],
            [
                (p[0] - self.anchors_[3]['anchor_pos'][0]) / est_ranges[3], 0.,
                (p[1] - self.anchors_[3]['anchor_pos'][1]) / est_ranges[3], 0.,
                (p[2] - self.anchors_[3]['anchor_pos'][2]) / est_ranges[3], 0.
            ]
        ])  # KINDA SUS

        self.get_logger().info(f"""
                               H {self.kalman_filter_.H}
                               """)

        # Filter predict and update
        self.kalman_filter_.predict()
        self.kalman_filter_.update([a['range'] for a in self.anchors_])

        # Sending the estimated position
        msg = PointStamped()
        msg.header.frame_id = self.get_namespace() + "/estimated_pos"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = self.kalman_filter_.x[0]
        msg.point.y = self.kalman_filter_.x[2]
        msg.point.z = self.kalman_filter_.x[4]
        self.est_pos_publisher_.publish(msg)

        self.get_logger().info(f"""{msg}""")


def main(args=None):
    rclpy.init(args=args)
    node = UwbEfk()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
