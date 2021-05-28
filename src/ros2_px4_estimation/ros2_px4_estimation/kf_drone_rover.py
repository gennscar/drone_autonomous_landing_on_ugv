#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from geometry_msgs.msg import PointStamped, Point
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry

class KfLoosePositioning(Node):
    """
    EKF
    """

    def __init__(self):
        super().__init__("kf_drone_rover")

        dT = 0.01

        # Kalman Filter
        self.kalman_filter_ = KalmanFilter(dim_x=18, dim_z=18)

        # State transition matrix
        f = np.array([
            [1., dT, 0.5*dT**2.],
            [0., 1.,         dT],
            [0., 0.,         1.]
        ])
        self.kalman_filter_.F = scipy.linalg.block_diag(*[f]*6)

        # Observation matrix
        self.kalman_filter_.H = np.eye(18)

        # Process noise
        Q1 = Q_discrete_white_noise(dim=3, dt=dT, var=1e-4, block_size=3)
        Q2 = Q_discrete_white_noise(dim=3, dt=dT, var=1e-4, block_size=3)
        self.kalman_filter_.Q = np.block([[Q1,np.zeros((9,9))],
                                         [np.zeros((9,9)),Q2]])

        # Covariance matrix
        self.kalman_filter_.P *= 50.

        # Setting up sensors subscribers
        self.uwb_sensor_subscriber_ = self.create_subscription(
            Point, "/drone_vehicle_uwb_positioning/target_coordinates", self.callback_uwb_subscriber, 10)
        self.px4_sensor_subscriber_ = self.create_subscription(
            VehicleLocalPosition, "/VehicleLocalPosition_PubSubTopic", self.callback_px4_subscriber, 10)
        self.px4_covariance_subscriber_ = self.create_subscription(
            VehicleOdometry, "/VehicleOdometry_PubSubTopic", self.callback_covariance_subscriber, 10)
        
        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PointStamped, "kf_target_coordinates", 10)

        # Prediction timer
        self.timer = self.create_timer(dT, self.predict_callback)

        self.get_logger().info("Node has started")

    def callback_uwb_subscriber(self, msg):
        # Storing current estimate in a np.array
        z = np.array([
            msg.x,
            msg.y,
            msg.z,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.
        ])

        R = 0.000625

        H = np.block([
            [-1., np.zeros((1,8)), 1., np.zeros((1,8))],
            [np.zeros((1,3)), -1., np.zeros((1,8)), 1., np.zeros((1,5))],
            [np.zeros((1,6)), -1., np.zeros((1,8)), 1., np.zeros((1,2))],
            [np.zeros((15,18))]
        ])

        # Filter update
        self.kalman_filter_.update(z, R, H)

    def callback_px4_subscriber(self, msg):
        # Storing current estimate in a np.array
        z = np.array([
            msg.y + 1.0,  msg.vy,  msg.ay,
            msg.x + 1.0,  msg.vx,  msg.ax,
            -msg.z,      -msg.vz, -msg.az,
            0.,           0.,      0.,
            0.,           0.,      0.,
            0.,           0.,      0.,
        ])

        R = 0.1

        H = np.block([
            [np.eye(9), np.zeros((9,9))],
            [np.zeros((9,18))]
        ])
        # Filter update
        self.kalman_filter_.update(z, R, H)

    def callback_covariance_subscriber(self, msg):
        pass

    def predict_callback(self):
        # Sending the estimated position
        msg = PointStamped()
        msg.header.frame_id = self.get_namespace() + "/estimated_pos"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = self.kalman_filter_.x[9][0] - self.kalman_filter_.x[0][0]
        msg.point.y = self.kalman_filter_.x[12][0] - self.kalman_filter_.x[3][0]
        msg.point.z = self.kalman_filter_.x[15][0] - self.kalman_filter_.x[6][0]
        self.est_pos_publisher_.publish(msg)

        self.kalman_filter_.predict()


def main(args=None):
    rclpy.init(args=args)
    node = KfLoosePositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
