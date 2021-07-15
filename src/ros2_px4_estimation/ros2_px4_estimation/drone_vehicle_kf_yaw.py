#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleOdometry


class KfLoosePositioning(Node):
    """
    EKF
    """

    def __init__(self):
        super().__init__("kf_drone_rover")
        
        self.declare_parameter('deltaT', 1.)
        self.declare_parameter('R_px4', 1.)
        self.declare_parameter('R_apriltag', 1.)
        self.declare_parameter('Q', 1.)
        self.declare_parameter('namespace_rover','')

        self.deltaT_ = self.get_parameter(
            'deltaT').get_parameter_value().double_value
        self.R_apriltag_ = self.get_parameter(
            'R_apriltag').get_parameter_value().double_value
        self.R_px4_ = self.get_parameter(
            'R_px4').get_parameter_value().double_value
        self.Q_ = self.get_parameter('Q').get_parameter_value().double_value
        self.namespace_rover = self.get_parameter('namespace_rover').get_parameter_value().string_value

        # Kalman Filter
        self.kalman_filter_ = KalmanFilter(dim_x=3, dim_z=3)

        # State transition matrix
        self.kalman_filter_.F = np.array([
            [1., self.deltaT_, 0.5*self.deltaT_**2.],
            [0., 1.,         self.deltaT_],
            [0., 0.,         1.]
        ])

        # Observation matrix
        self.kalman_filter_.H = np.eye(3)

        # Process noise
        self.kalman_filter_.Q = Q_discrete_white_noise(dim=3, dt=self.deltaT_, var=self.Q_, block_size=3)
        # Covariance matrix
        self.kalman_filter_.P *= 1.

        # Setting up sensors subscribers

        self.px4_odometry_subscriber = self.create_subscription(
            VehicleOdometry, self.namespace_rover + "/VehicleOdometry_PubSubTopic", self.callback_rover_px4_subscriber, 10)
        
        self.apriltag_subscriber = self.create_subscription(
            VehicleLocalPosition, self.namespace_rover + "/VehicleOdometry_PubSubTopic", self.callback_rover_px4_subscriber, 10)
        
        # Setting up position publisher
        #self.est_pos_publisher_ = self.create_publisher(
        #    PoseWithCovarianceStamped, self.get_namespace() + "/estimated_pos", 10)

        # Prediction timer
        self.timer = self.create_timer(self.deltaT_, self.predict_callback)

        self.get_logger().info("Node has started")

    def callback_uwb_subscriber(self, msg):
        # Storing current estimate in a np.array
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
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
            0.,
            0.
        ])


        H = np.block([
            [-1., np.zeros((1, 8)), 1., np.zeros((1, 8))],
            [np.zeros((1, 3)), -1., np.zeros((1, 8)), 1., np.zeros((1, 5))],
            [np.zeros((16, 18))]
        ])

        # Filter update
        self.kalman_filter_.update(z, self.R_uwb_, H)
    def callback_drone_px4_subscriber(self, msg):
        if self.include_drone == 1:
            # Storing current estimate in a np.array
            z = np.array([
                msg.y,  msg.vy,  msg.ay,
                msg.x,  msg.vx,  msg.ax,
                -msg.z,      -msg.vz, -msg.az,
                0.,           0.,      0.,
                0.,           0.,      0.,
                0.,           0.,      0.,
            ])

            H = np.block([
                [np.eye(9), np.zeros((9, 9))],
                [np.zeros((9, 18))]
            ])
            # Filter update
            self.kalman_filter_.update(z, self.R_px4_, H)
        else:
            pass

    def callback_rover_px4_subscriber(self, msg):
        if self.include_rover == 1:
            # Storing current estimate in a np.array
            z = np.array([
                0.,  msg.vy,  msg.ay,
                0.,  msg.vx,  msg.ax,
                0., -msg.vz, -msg.az,
                0.,  0.,      0.,
                0.,  0.,      0.,
                0.,  0.,      0.])      
            block_matrix_1 = np.array([[0., 0., 0.],
                                    [0., 1., 0.],
                                    [0., 0., 1.]])        
            block_matrix_2 = np.block([
                            [block_matrix_1, np.zeros((3,3)), np.zeros((3,3))],
                            [np.zeros((3,3)), block_matrix_1, np.zeros((3,3))],
                            [np.zeros((3,3)), np.zeros((3,3)), block_matrix_1]
            ])
            H = np.block([
                [np.zeros((9, 9)), block_matrix_2],
                [np.zeros((9, 18))]
            ])
            # Filter update
            self.kalman_filter_.update(z, self.R_px4_, H)
        else:
            pass

    def predict_callback(self):
        # Sending the estimated position
        #msg = PoseWithCovarianceStamped()
        #msg.header.frame_id = self.get_namespace() + "/estimated_pos"
        #msg.header.stamp = self.get_clock().now().to_msg()
        #msg.pose.pose.position.x = float(self.kalman_filter_.x[9][0] - self.kalman_filter_.x[0][0])
        #msg.pose.pose.position.y = float(self.kalman_filter_.x[12][0] - self.kalman_filter_.x[3][0])
        #msg.pose.pose.position.z = float(self.kalman_filter_.x[15][0] - self.kalman_filter_.x[6][0])
        #self.est_pos_publisher_.publish(msg)

        self.kalman_filter_.predict()


def main(args=None):
    rclpy.init(args=args)
    node = KfLoosePositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
