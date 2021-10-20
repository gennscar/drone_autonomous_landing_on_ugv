#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import scipy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import Range
from scipy.spatial.transform import Rotation as R

from ros2_px4_interfaces.msg import Yaw

class kf_xyz_estimator(Node):

    def __init__(self):
        super().__init__("kf_drone_rover")

        self.watchdog_counter = np.zeros((3,1))
        self.watchdog_dT_ = 1.0
        self.run_publisher = True
        
        # Parameters
        self.declare_parameter('deltaT', 1e-1)
        self.declare_parameter('R_uwb', 1e-3)
        self.declare_parameter('R_px4', 1e-2)
        self.declare_parameter("R_range_sensor", 1e-2)
        self.declare_parameter("R_compass", 1e-2)
        self.declare_parameter("R_apriltag", 1e-2)
        self.declare_parameter("rng_sensor_fuse_radius", 0.4)
        self.declare_parameter("rng_sensor_out_radius", 2.0)
        self.declare_parameter("rng_sensor_max_height", 10.0)
        self.declare_parameter("rng_sensor_min_height", 0.2)
        self.declare_parameter('Q_drone', 1e-3)
        self.declare_parameter('Q_drone_z', 1e-3)
        self.declare_parameter('Q_rover', 1e-3)
        self.declare_parameter('Q_rover_z', 1e-3)
        self.declare_parameter('Q_compass', 1e-3)
        self.declare_parameter('Q_bias', 1e-3)

        self.declare_parameter('vehicle_namespace','/drone')
        self.declare_parameter("uwb_estimator", "/LS_drone_rover_uwb_estimator/norot_pos")
        self.declare_parameter("yaw_subscriber_topic", "/yaw_sensor/estimated_yaw")
        self.declare_parameter("enable_watchdog", True)

        self.deltaT_ = self.get_parameter('deltaT').get_parameter_value().double_value
        self.R_uwb_ = self.get_parameter('R_uwb').get_parameter_value().double_value
        self.R_px4_ = self.get_parameter('R_px4').get_parameter_value().double_value
        self.R_range_sensor_ = self.get_parameter('R_range_sensor').get_parameter_value().double_value
        self.R_compass_ = self.get_parameter('R_compass').get_parameter_value().double_value
        self.R_apriltag_ = self.get_parameter('R_apriltag').get_parameter_value().double_value
        self.rng_sensor_fuse_radius_ = self.get_parameter('rng_sensor_fuse_radius').get_parameter_value().double_value
        self.rng_sensor_out_radius_ = self.get_parameter('rng_sensor_out_radius').get_parameter_value().double_value
        self.rng_sensor_max_height_ = self.get_parameter('rng_sensor_max_height').get_parameter_value().double_value
        self.rng_sensor_min_height_ = self.get_parameter('rng_sensor_min_height').get_parameter_value().double_value
        self.Q_drone = self.get_parameter('Q_drone').get_parameter_value().double_value
        self.Q_drone_z = self.get_parameter('Q_drone_z').get_parameter_value().double_value
        self.Q_rover = self.get_parameter('Q_rover').get_parameter_value().double_value
        self.Q_rover_z= self.get_parameter('Q_rover_z').get_parameter_value().double_value
        self.Q_compass= self.get_parameter('Q_compass').get_parameter_value().double_value
        self.Q_bias= self.get_parameter('Q_bias').get_parameter_value().double_value

        self.vehicle_namespace = self.get_parameter('vehicle_namespace').get_parameter_value().string_value
        self.uwb_estimator = self.get_parameter("uwb_estimator").get_parameter_value().string_value
        self.yaw_subscriber_topic = self.get_parameter("yaw_subscriber_topic").get_parameter_value().string_value
        self.enable_watchdog = self.get_parameter("enable_watchdog").get_parameter_value().bool_value

        # Kalman Filter
        self.kalman_filter_ = KalmanFilter(dim_x=15, dim_z=15)

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
        self.kalman_filter_.F = np.block([[scipy.linalg.block_diag(*[f_1_]*2), np.zeros((6,9))],
                                          [np.zeros((2,6)),  f_2_, np.zeros((2,7))],
                                          [np.zeros((4,8)),  scipy.linalg.block_diag(*[f_2_]*2), np.zeros((4,3))],
                                          [np.zeros((1,12)), 1., 0., 0.],
                                          [np.zeros((1,12)), 0., 1., 0.],
                                          [np.zeros((1,12)), 0., 0., 1.]])

        # Process noise
        Q1_1 = Q_discrete_white_noise(dim=3, dt=self.deltaT_, var=self.Q_drone, block_size=2)
        Q1_2 = Q_discrete_white_noise(dim=2, dt=self.deltaT_, var=self.Q_drone_z, block_size=1)
        Q2 = Q_discrete_white_noise(dim=2, dt=self.deltaT_, var=self.Q_rover, block_size=2)
        self.kalman_filter_.Q = np.block([[Q1_1, np.zeros((6,9))],
                                          [np.zeros((2,6)), Q1_2, np.zeros((2,7))],
                                          [np.zeros((4,8)), Q2,  np.zeros((4,3))],
                                          [np.zeros((1,12)), self.Q_rover_z*0.25*self.deltaT_**4, 0., 0.],
                                          [np.zeros((1,12)), 0., self.Q_compass*0.25*self.deltaT_**4, 0.],
                                          [np.zeros((1,12)), 0., 0., self.Q_bias*0.25*self.deltaT_**4]])

        # Covariance matrix
        self.kalman_filter_.P *= 1.

        # Setting up sensors subscribers
        self.target_uwb_position_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, self.uwb_estimator, self.callback_uwb_subscriber, 3)
        self.px4_drone_subscriber = self.create_subscription(
            VehicleLocalPosition, self.vehicle_namespace + "/VehicleLocalPosition_PubSubTopic", self.callback_px4_drone_subscriber, 10)
        self.range_sensor_subscriber = self.create_subscription(
            Range, self.vehicle_namespace + "/DistanceSensor_PubSubTopic", self.callback_range_sensor_subscriber, 10)
        self.range_sensor_subscriber = self.create_subscription(
            Odometry, "/AprilTag_estimator/estimated_pos", self.callback_apriltag_subscriber, 10)
        self.compass_subscriber = self.create_subscription(
            Yaw, self.yaw_subscriber_topic, self.callback_compass_subscriber, 10)
            
        # Setting up position and velocity publisher
        self.est_pos_publisher = self.create_publisher(
            Odometry, self.get_namespace() + "/estimated_pos", 10)

        # Prediction timer
        self.timer = self.create_timer(self.deltaT_, self.predict_callback)

        # Watchdog timer
        if self.enable_watchdog:
            self.watchdog_timer = self.create_timer(self.watchdog_dT_, self.watchdog_callback)

        # Printing node started
        self.get_logger().info("Node has started")

        # Initialization variables
        self.rel_pos_x_ = []
        self.rel_pos_y_ = []
        self.rel_pos_z_ = []
        self.norm_rel_xy_pos = -1.0

    def callback_uwb_subscriber(self, msg):

        corrected_angle_ = self.kalman_filter_.x[13][0] + self.kalman_filter_.x[14][0]
        #corrected_angle_ = self.kalman_filter_.x[13][0]

        rover_inv_rotation = (R.from_euler(
            'z', corrected_angle_ , degrees=True))
        rover_rotation = R.inv(rover_inv_rotation)
        norot_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, 0.])
        ENU_pos = rover_rotation.apply(norot_pos)
        # Storing current estimate in a np.array
        z = np.array([
            ENU_pos[0],
            - ENU_pos[1],
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
            [1., np.zeros((1, 7)), -1., np.zeros((1, 6))],
            [np.zeros((1, 3)), 1., np.zeros((1, 6)), -1., np.zeros((1, 4))],
            [np.zeros((13, 15))]
        ])

        # Filter update
        self.kalman_filter_.update(z, self.R_uwb_, H)

        self.watchdog_counter[0][0] += 1

    def callback_px4_drone_subscriber(self, msg):
        z = np.array([
            msg.x,
            msg.vx,
            msg.ax,
            msg.y,
            msg.vy,
            msg.ay,
            -msg.z,
            -msg.vz,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.
        ])

        H = np.block([
            [np.eye(8), np.zeros((8, 7))],
            [np.zeros((7, 15))]
        ])
        # Filter update
        self.kalman_filter_.update(z, self.R_px4_, H)

        self.watchdog_counter[1][0] += 1

    def callback_range_sensor_subscriber(self, msg):

        if self.norm_rel_xy_pos != -1.0 and \
           self.norm_rel_xy_pos <= self.rng_sensor_fuse_radius_ and \
           msg.range <= self.rng_sensor_max_height_ and \
           msg.range >= self.rng_sensor_min_height_:
            
            z = np.array([
                msg.range,
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
                [np.zeros((1, 6)), 1., np.zeros((1, 5)), -1., np.zeros((1,2))],
                [np.zeros((14, 15))]
            ])
            # Filter update
            self.kalman_filter_.update(z, self.R_range_sensor_, H)

        elif self.norm_rel_xy_pos != -1.0 and \
             self.norm_rel_xy_pos >= self.rng_sensor_out_radius_ and \
             msg.range <= self.rng_sensor_max_height_ and \
             msg.range >= self.rng_sensor_min_height_:

            z = np.array([
                msg.range,
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
                [np.zeros((1, 6)), 1., np.zeros((1, 8))],
                [np.zeros((14, 15))]
            ])
            # Filter update
            self.kalman_filter_.update(z, self.R_range_sensor_, H)
            
        else:
            pass

    def callback_compass_subscriber(self, msg):

        z = np.array([
            msg.yaw,
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
            [np.zeros((1,13)), 1., 0.],
            [np.zeros((14, 15))]
        ])


        # Filter update
        self.kalman_filter_.update(z, self.R_compass_, H)

        self.watchdog_counter[2][0] += 1

    def callback_apriltag_subscriber(self, msg):

        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.w - float(self.kalman_filter_.x[13][0]),
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
            [1., np.zeros((1, 7)), -1., np.zeros((1, 6))],
            [np.zeros((1, 3)), 1., np.zeros((1, 6)), -1., np.zeros((1, 4))],
            [np.zeros((1, 6)), 1., np.zeros((1, 5)), -1., np.zeros((1, 2))],
            [np.zeros((1,13)), 0., 1.],
            [np.zeros((11,15))]
        ])
        
        # Filter update
        R_ = np.eye(15)
        R_[0][0] = self.R_apriltag_
        R_[1][1] = self.R_apriltag_
        R_[2][2] = self.R_apriltag_
        R_[3][3] = 1. 
        self.kalman_filter_.update(z, R_, H)


    def predict_callback(self):

        if not self.run_publisher:
            return

        self.rel_pos_x_ = float(- self.kalman_filter_.x[8][0] + self.kalman_filter_.x[0][0])
        self.rel_pos_y_ = float(- self.kalman_filter_.x[10][0] + self.kalman_filter_.x[3][0])
        self.rel_pos_z_ = float(- self.kalman_filter_.x[12][0] + self.kalman_filter_.x[6][0])
        self.norm_rel_xy_pos = np.linalg.norm([self.rel_pos_x_, self.rel_pos_y_], ord=2)
        self.rel_vel_x_ = float(- self.kalman_filter_.x[9][0] + self.kalman_filter_.x[1][0])
        self.rel_vel_y_ = float(- self.kalman_filter_.x[11][0] + self.kalman_filter_.x[4][0])
        self.rel_vel_z_ = float(+ self.kalman_filter_.x[7][0])

        # Sending the estimated pose
        msg = Odometry()
        msg.header.frame_id = self.get_namespace() + "/estimated_pos"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.rel_pos_x_
        msg.pose.pose.position.y = self.rel_pos_y_
        msg.pose.pose.position.z = self.rel_pos_z_
        msg.pose.pose.orientation.w = float(self.kalman_filter_.x[13][0])
        msg.twist.twist.linear.x = self.rel_vel_x_
        msg.twist.twist.linear.y = self.rel_vel_y_
        msg.twist.twist.linear.z = self.rel_vel_z_
        self.est_pos_publisher.publish(msg)

        # Predict
        self.kalman_filter_.predict()

    def watchdog_callback(self):

        if any(self.watchdog_counter == 0):
            self.run_publisher = False
            self.get_logger().warn(f"No sensor data available, not publishing.")
        else:
            self.run_publisher = True
        self.watchdog_counter = np.zeros((3,1))

def main(args=None):
    rclpy.init(args=args)
    node = kf_xyz_estimator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
