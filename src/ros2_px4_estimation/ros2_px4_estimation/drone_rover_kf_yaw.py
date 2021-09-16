#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy 
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from px4_msgs.msg import VehicleOdometry
from ros2_px4_interfaces.msg import Yaw
from scipy.spatial.transform import Rotation as R

class KFYawEstimator(Node):
    """
    EKF
    """

    def __init__(self):
        super().__init__("kf_drone_rover")
        
        self.declare_parameter('deltaT', 1.)
        self.declare_parameter('R_px4_yaw', 1.)
        self.declare_parameter('R_apriltag', 1.)
        self.declare_parameter('Q', 1.)
        self.declare_parameter('namespace_rover','/rover')

        self.deltaT_ = self.get_parameter(
            'deltaT').get_parameter_value().double_value
        self.R_apriltag_ = self.get_parameter(
            'R_apriltag').get_parameter_value().double_value
        self.R_px4_yaw_ = self.get_parameter(
            'R_px4_yaw').get_parameter_value().double_value
        self.Q_ = self.get_parameter('Q').get_parameter_value().double_value
        self.namespace_rover = self.get_parameter('namespace_rover').get_parameter_value().string_value

        # Kalman Filter
        self.kalman_filter_ = KalmanFilter(dim_x=3, dim_z=3)

        # State transition matrix
        f = np.array([
            [1., self.deltaT_, 0.5*self.deltaT_**2.],
            [0., 1.,         self.deltaT_],
            [0., 0.,         1.]
        ])
        self.kalman_filter_.F = f
        # Observation matrix
        self.kalman_filter_.H = np.eye(3)

        # Process noise
        Q1 = Q_discrete_white_noise(dim=3, dt=self.deltaT_, var=self.Q_, block_size=1)
        self.kalman_filter_.Q = Q1
        # Covariance matrix
        self.kalman_filter_.P *= 1.

        # Setting up sensors subscribers

        self.px4_yaw_subscriber = self.create_subscription(
            Yaw, "/px4_estimator/estimated_yaw", self.callback_px4_yaw_subscriber, 10)
        self.apriltag_subscriber = self.create_subscription(
            Yaw, "/AprilTag_estimator/estimated_yaw", self.callback_apriltag_subscriber, 10)
        # Setting up yaw publisher
        self.est_yaw_publisher_ = self.create_publisher(
            Yaw, self.get_namespace() + "/estimated_yaw", 10)
        # Prediction timer
        self.timer = self.create_timer(self.deltaT_, self.predict_callback)

        self.get_logger().info("Node has started")


    def callback_apriltag_subscriber(self, msg):
        # Storing current estimate in a np.array
        z = np.array([
            msg.yaw,  0., 0.
        ])

        H = np.array([[1., 0., 0.],
                     [0., 0., 0.],
                     [0., 0., 0.]])

        # Filter update
        self.kalman_filter_.update(z, self.R_apriltag_, H)


    def callback_px4_yaw_subscriber(self, msg):

        # Storing current estimate in a np.array
        z = np.array([
            msg.yaw,  0., 0.
        ])

        H = np.array([[1., 0., 0.],
                     [0., 0., 0.],
                     [0., 0., 0.]])

        R_px4_yaw = self.R_px4_yaw_
        # Filter update
        self.kalman_filter_.update(z, R_px4_yaw, H)


    def predict_callback(self):
        # Sending the estimated yaw
        msg = Yaw()
        msg.yaw = self.kalman_filter_.x[0][0]
        msg.header.frame_id = self.get_namespace() + "/estimated_yaw"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.est_yaw_publisher_.publish(msg)

        self.kalman_filter_.predict()


def main(args=None):
    rclpy.init(args=args)
    node = KFYawEstimator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
