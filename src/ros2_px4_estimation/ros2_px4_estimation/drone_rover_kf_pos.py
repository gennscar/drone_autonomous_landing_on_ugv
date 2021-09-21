#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from px4_msgs.msg import VehicleLocalPosition


class KfConstHeight(Node):

    def __init__(self):
        super().__init__("kf_drone_rover")
        
        # Parameters
        self.declare_parameter('deltaT', 1e-1)
        self.declare_parameter('R_uwb', 1e-3)
        self.declare_parameter('R_px4', 1e-2)
        self.declare_parameter("R_range_sensor", 1e-2)
        self.declare_parameter("rng_sensor_fuse_radius", 0.4)
        self.declare_parameter("rng_sensor_out_radius", 2.0)
        self.declare_parameter('Q_drone', 1e-3)
        self.declare_parameter('Q_drone_z', 1e-3)
        self.declare_parameter('Q_rover', 1e-3)
        self.declare_parameter('Q_rover_z', 1e-3)
        self.declare_parameter('namespace_drone','/drone')
        self.declare_parameter('include_drone',1)
        self.declare_parameter("uwb_estimator", "/LS_uwb_estimator")
        self.deltaT_ = self.get_parameter('deltaT').get_parameter_value().double_value
        self.R_uwb_ = self.get_parameter('R_uwb').get_parameter_value().double_value
        self.R_px4_ = self.get_parameter('R_px4').get_parameter_value().double_value
        self.R_range_sensor = self.get_parameter('R_range_sensor').get_parameter_value().double_value
        self.rng_sensor_fuse_radius_ = self.get_parameter('rng_sensor_fuse_radius').get_parameter_value().double_value
        self.rng_sensor_out_radius_ = self.get_parameter('rng_sensor_out_radius').get_parameter_value().double_value
        self.Q_drone = self.get_parameter('Q_drone').get_parameter_value().double_value
        self.Q_drone_z = self.get_parameter('Q_drone_z').get_parameter_value().double_value
        self.Q_rover = self.get_parameter('Q_rover').get_parameter_value().double_value
        self.Q_rover_z= self.get_parameter('Q_rover_z').get_parameter_value().double_value
        self.namespace_drone = self.get_parameter('namespace_drone').get_parameter_value().string_value
        self.include_drone = self.get_parameter('include_drone').get_parameter_value().integer_value
        self.uwb_estimator = self.get_parameter("uwb_estimator").get_parameter_value().string_value

        # Kalman Filter
        self.kalman_filter_ = KalmanFilter(dim_x=13, dim_z=13)

        # State transition matrix
        f_1_ = np.array([
            [1., self.deltaT_, 0.5*self.deltaT_**2.],
            [0., 1.,         self.deltaT_],
            [0., 0.,         1.]
        ])
        f_2_ = np.array([
            [1., self.deltaT_],
            [0., 1.]
        ])
        self.kalman_filter_.F = np.block([[scipy.linalg.block_diag(*[f_1_]*2), np.zeros((6,7))],
                                          [np.zeros((2,6)),  f_2_, np.zeros((2,5))],
                                          [np.zeros((4,8)),  scipy.linalg.block_diag(*[f_2_]*2), np.zeros((4,1))],
                                          [np.zeros((1,12)), 1.]])

        # Process noise
        Q1_1 = Q_discrete_white_noise(dim=3, dt=self.deltaT_, var=self.Q_drone, block_size=2)
        Q1_2 = Q_discrete_white_noise(dim=2, dt=self.deltaT_, var=self.Q_drone_z, block_size=1)
        Q2 = Q_discrete_white_noise(dim=2, dt=self.deltaT_, var=self.Q_rover, block_size=2)
        self.kalman_filter_.Q = np.block([[Q1_1, np.zeros((6,7))],
                                          [np.zeros((2,6)), Q1_2, np.zeros((2,5))],
                                          [np.zeros((4,8)), Q2,  np.zeros((4,1))],
                                          [np.zeros((1,12)), self.Q_rover_z*0.25*self.deltaT_**4]])

        # Covariance matrix
        self.kalman_filter_.P *= 10.

        # Setting up sensors subscribers
        self.target_uwb_position_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, self.uwb_estimator + "/estimated_pos", self.callback_uwb_subscriber, 3)
        self.px4_drone_subscriber = self.create_subscription(
            VehicleLocalPosition, self.namespace_drone + "/VehicleLocalPosition_PubSubTopic", self.callback_px4_drone_subscriber, 10)

        self.range_sensor_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, "/range_sensor_positioning/estimated_pos", self.callback_range_sensor_subscriber, 10)

            
        # Setting up position and velocity publisher
        self.est_pos_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, self.get_namespace() + "/estimated_pos", 10)
        self.est_vel_publisher_ = self.create_publisher(
            TwistWithCovarianceStamped, self.get_namespace() + "/estimated_vel", 10)

        # Prediction timer
        self.timer = self.create_timer(self.deltaT_, self.predict_callback)

        # Printing node started
        self.get_logger().info("Node has started")

        # Initialization variables
        self.rel_pos_x_ = []
        self.rel_pos_y_ = []
        self.rel_pos_z_ = []
        self.norm_rel_xy_pos = -1.0

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
            0.
        ])


        H = np.block([
            [1., np.zeros((1, 7)), -1., np.zeros((1, 4))],
            [np.zeros((1, 3)), 1., np.zeros((1, 6)), -1., np.zeros((1, 2))],
            [np.zeros((11, 13))]
        ])

        # Filter update
        self.kalman_filter_.update(z, self.R_uwb_, H)

    def callback_px4_drone_subscriber(self, msg):
        if self.include_drone == 1:
            z = np.array([
                msg.y,
                msg.vy,
                msg.ay,
                msg.x,
                msg.vx,
                msg.ax,
                -msg.z,
                -msg.vz,
                0.,
                0.,
                0.,
                0.,
                0.
            ])

            H = np.block([
                [np.eye(8), np.zeros((8, 5))],
                [np.zeros((5, 13))]
            ])
            # Filter update
            self.kalman_filter_.update(z, self.R_px4_, H)
        else:
            pass

    def callback_range_sensor_subscriber(self, msg):

        if self.norm_rel_xy_pos != -1.0 and \
           self.norm_rel_xy_pos <= self.rng_sensor_fuse_radius_:
            
            z = np.array([
                msg.pose.pose.position.z,
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
                [np.zeros((1, 6)), 1., np.zeros((1, 5)), -1.],
                [np.zeros((12, 13))]
            ])
            # Filter update
            self.kalman_filter_.update(z, self.R_range_sensor, H)

        elif self.norm_rel_xy_pos != -1.0 and \
             self.norm_rel_xy_pos >= self.rng_sensor_out_radius_:

            z = np.array([
                msg.pose.pose.position.z,
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
                [np.zeros((1, 6)), 1., np.zeros((1, 6))],
                [np.zeros((12, 13))]
            ])
            # Filter update
            self.kalman_filter_.update(z, self.R_range_sensor, H)
            
        else:
            pass

    def predict_callback(self):

        self.rel_pos_x_ = float(- self.kalman_filter_.x[8][0] + self.kalman_filter_.x[0][0])
        self.rel_pos_y_ = float(- self.kalman_filter_.x[10][0] + self.kalman_filter_.x[3][0])
        self.rel_pos_z_ = float(- self.kalman_filter_.x[12][0] + self.kalman_filter_.x[6][0])
        self.norm_rel_xy_pos = np.linalg.norm([self.rel_pos_x_, self.rel_pos_y_], ord=2)

        # Sending the estimated position
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.get_namespace() + "/estimated_pos"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.rel_pos_x_
        msg.pose.pose.position.y = self.rel_pos_y_
        msg.pose.pose.position.z = self.rel_pos_z_
        self.est_pos_publisher_.publish(msg)

        self.rel_vel_x_ = float(- self.kalman_filter_.x[9][0] + self.kalman_filter_.x[1][0])
        self.rel_vel_y_ = float(- self.kalman_filter_.x[11][0] + self.kalman_filter_.x[4][0])
        self.rel_vel_z_ = float(+ self.kalman_filter_.x[7][0])

        # Sending the estimated velocity
        msg = TwistWithCovarianceStamped()
        msg.header.frame_id = self.get_namespace() + "/estimated_vel"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.twist.linear.x = self.rel_vel_x_
        msg.twist.twist.linear.y = self.rel_vel_y_
        msg.twist.twist.linear.z = self.rel_vel_z_
        self.est_vel_publisher_.publish(msg)

        # Predict
        self.kalman_filter_.predict()


def main(args=None):
    rclpy.init(args=args)
    node = KfConstHeight()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
