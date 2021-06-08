#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleLocalPosition


class KfLoosePositioning(Node):
    """
    EKF
    """

    def __init__(self):
        super().__init__("kf_loose_positioning")

        self.declare_parameter('deltaT', 1.)
        self.declare_parameter('R_uwb', 1.)
        self.declare_parameter('R_px4', 1.)
        self.declare_parameter('Q', 1.)

        self.deltaT_ = self.get_parameter(
            'deltaT').get_parameter_value().double_value
        self.R_uwb_ = self.get_parameter(
            'R_uwb').get_parameter_value().double_value
        self.R_px4_ = self.get_parameter(
            'R_px4').get_parameter_value().double_value
        self.Q_ = self.get_parameter('Q').get_parameter_value().double_value

        # Kalman Filter
        self.kalman_filter_ = KalmanFilter(dim_x=9, dim_z=9)

        # State transition matrix
        f = np.array([
            [1., self.deltaT_, 0.5*self.deltaT_**2.],
            [0., 1.,         self.deltaT_],
            [0., 0.,         1.]
        ])
        self.kalman_filter_.F = scipy.linalg.block_diag(*[f]*3)

        # Process noise
        self.kalman_filter_.Q = Q_discrete_white_noise(
            dim=3, dt=self.deltaT_, var=self.Q_, block_size=3)

        # Covariance matrix
        self.kalman_filter_.P *= 10.

        # Setting up sensors subscribers
        self.sensor_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, "/LS_uwb_estimator/estimated_pos", self.callback_uwb_subscriber, 10)
        self.sensor_subscriber_ = self.create_subscription(
            VehicleLocalPosition, "/VehicleLocalPosition_PubSubTopic", self.callback_px4_subscriber, 10)

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, self.get_namespace() + "/estimated_pos", 10)

        # Prediction timer
        self.timer = self.create_timer(self.deltaT_, self.predict_callback)

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
        self.kalman_filter_.update(z, self.R_uwb_, H)

    def callback_px4_subscriber(self, msg):
        # Storing current estimate in a np.array
        z = np.array([
            msg.y + 1.0,  msg.vy,  msg.ay,
            msg.x + 1.0,  msg.vx,  msg.ax,
            -msg.z,      -msg.vz, -msg.az
        ])

        # Filter update
        self.kalman_filter_.update(z, self.R_px4_, H=np.eye(9))

    def predict_callback(self):
        # Sending the estimated position
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.get_namespace() + "/estimated_pos"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.kalman_filter_.x[0][0]
        msg.pose.pose.position.y = self.kalman_filter_.x[3][0]
        msg.pose.pose.position.z = self.kalman_filter_.x[6][0]

        msg.pose.covariance[0] = self.kalman_filter_.P[0][0]
        msg.pose.covariance[1] = self.kalman_filter_.P[0][1]
        msg.pose.covariance[2] = self.kalman_filter_.P[0][2]
        msg.pose.covariance[3] = self.kalman_filter_.P[1][0]
        msg.pose.covariance[4] = self.kalman_filter_.P[1][1]
        msg.pose.covariance[5] = self.kalman_filter_.P[1][2]
        msg.pose.covariance[6] = self.kalman_filter_.P[2][0]
        msg.pose.covariance[7] = self.kalman_filter_.P[2][1]
        msg.pose.covariance[8] = self.kalman_filter_.P[2][2]

        self.est_pos_publisher_.publish(msg)

        self.kalman_filter_.predict()


def main(args=None):
    rclpy.init(args=args)
    node = KfLoosePositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
