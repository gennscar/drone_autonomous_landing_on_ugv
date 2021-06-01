#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy

from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise

from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import UwbSensor
from px4_msgs.msg import VehicleLocalPosition


class UkfPositioning(Node):
    """
    EKF
    """

    def __init__(self):
        super().__init__("kf_tight_positioning")

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

        self.predicted_ = False

        # Kalman Filter
        sigmas = MerweScaledSigmaPoints(
            9, alpha=1e-3, beta=2., kappa=0., sqrt_method=scipy.linalg.sqrtm)

        def f_(x, dt):
            f = np.array([
                [1., dt, 0.5*dt**2.],
                [0., 1.,         dt],
                [0., 0.,         1.]
            ])
            F = scipy.linalg.block_diag(*[f]*3)
            return F @ x

        self.kalman_filter_ = UKF(
            dim_x=9, dim_z=9, dt=self.deltaT_, fx=f_, hx=None, points=sigmas)

        # Initial estimate
        self.kalman_filter_.x = np.array(
            [1., 0., 0., 1., 0., 0., 1., 0., 0.])

        # State transition matrix

        # Covariance matrix
        self.kalman_filter_.P *= 10.

        # Process noise
        self.kalman_filter_.Q = Q_discrete_white_noise(
            dim=3, dt=self.deltaT_, var=self.Q_, block_size=3)

        # Setting up sensors subscribers
        self.sensor_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor_0", self.callback_uwb_subscriber, 10)
        self.sensor_subscriber_ = self.create_subscription(
            VehicleLocalPosition, "/VehicleLocalPosition_PubSubTopic", self.callback_px4_subscriber, 10)

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, self.get_namespace() + "/estimated_pos", 10)

        # Prediction timer
        self.timer = self.create_timer(self.deltaT_, self.predict_callback)

        self.get_logger().info(f"""Node has started
                               deltaT:  {self.deltaT_}
                               R_uwb:   {self.R_uwb_}
                               R_px4:   {self.R_px4_}
                               Q:       {self.Q_}
                               """)

    def callback_uwb_subscriber(self, msg):
        if(not self.predicted_):
            return

        # Storing measurement in a np.array
        z = np.array([msg.range, 0., 0., 0., 0., 0., 0., 0., 0.])

        # Storing anchor position in a np.array
        anc_pos = np.array([
            msg.anchor_pos.x,
            msg.anchor_pos.y,
            msg.anchor_pos.z
        ])

        def h_uwb(x):
            r = np.linalg.norm(x[[0, 3, 6]] - anc_pos)
            return np.array([r, 0., 0., 0., 0., 0., 0., 0., 0.])

        # Filter update
        self.kalman_filter_.update(z, R=self.R_uwb_, hx=h_uwb)

    def callback_px4_subscriber(self, msg):
        if(not self.predicted_):
            return

        # Storing measurement in a np.array
        z = np.array([
            msg.y + 1.0,  msg.vy,  msg.ay,
            msg.x + 1.0,  msg.vx,  msg.ax,
            -msg.z,      -msg.vz, -msg.az
        ])

        def h_px4(x):
            return x

        # Filter update
        self.kalman_filter_.update(z, R=self.R_px4_, hx=h_px4)

    def predict_callback(self):
        # Sending the estimated position
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.get_namespace() + "/estimated_pos"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.kalman_filter_.x[0]
        msg.pose.pose.position.y = self.kalman_filter_.x[3]
        msg.pose.pose.position.z = self.kalman_filter_.x[6]

        msg.pose.covariance[0] = self.kalman_filter_.P[0][0]
        msg.pose.covariance[1] = self.kalman_filter_.P[0][3]
        msg.pose.covariance[2] = self.kalman_filter_.P[0][6]
        msg.pose.covariance[3] = self.kalman_filter_.P[3][0]
        msg.pose.covariance[4] = self.kalman_filter_.P[3][3]
        msg.pose.covariance[5] = self.kalman_filter_.P[3][6]
        msg.pose.covariance[6] = self.kalman_filter_.P[6][0]
        msg.pose.covariance[7] = self.kalman_filter_.P[6][3]
        msg.pose.covariance[8] = self.kalman_filter_.P[6][6]

        self.est_pos_publisher_.publish(msg)

        # Filter predict
        self.kalman_filter_.predict()
        self.predicted_ = True


def main(args=None):
    rclpy.init(args=args)
    node = UkfPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
