#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from px4_msgs.msg import VehicleVisualOdometry, Timesync


class UwbEstimate2Px4(Node):
    """[summary]

    Args:
        Node ([type]): [description]
    """

    def __init__(self):
        super().__init__("uwb_estimate_2_px4")

        self.timestamp_ = 0

        # Position and velocity estimate + covariance matrix
        self.x_ = np.zeros(6)
        self.cov_ = np.eye(6)

        # Name of the estimator node
        self.estimator_name_ = self.declare_parameter("estimator_name", "")

        # Retrieve parameter values
        self.estimator_name_ = self.get_parameter(
            "estimator_name").get_parameter_value().string_value

        # Parameter check
        if(self.estimator_name_ == ""):
            self.get_logger().error("uwb_estimate_2_px4 needs an estimator name")
            self.destroy_node()

        # Setting up subscriber for the UWB estimate
        self.position_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, self.estimator_name_ + "/estimated_pos", self.callback_estimator_pos, 10)
        self.velocity_subscriber_ = self.create_subscription(
            TwistWithCovarianceStamped, self.estimator_name_ + "/estimated_vel", self.callback_estimator_vel, 10)

        # Retrieve timestamp from PX4
        self.timesync_subscriber_ = self.create_subscription(
            Timesync, "/Timesync_PubSubTopic", self.callback_timesync, 10)

        # Timer and publisher to send the estimate to PX4
        self.timer_ = self.create_timer(0.1, self.callback_send_odom)
        self.px4_odometry_publisher_ = self.create_publisher(
            VehicleVisualOdometry, "/VehicleVisualOdometry_PubSubTopic", 10)

        self.get_logger().info(f"""Node has started:
                               estimator_name: {self.estimator_name_}
                               """)

    def callback_estimator_pos(self, msg):
        self.x_[0] = msg.pose.pose.position.x
        self.x_[1] = msg.pose.pose.position.y
        self.x_[2] = msg.pose.pose.position.z

        self.cov_[0][0] = msg.pose.covariance[0]
        self.cov_[0][1] = msg.pose.covariance[1]
        self.cov_[0][2] = msg.pose.covariance[2]
        self.cov_[1][0] = msg.pose.covariance[6]
        self.cov_[1][1] = msg.pose.covariance[7]
        self.cov_[1][2] = msg.pose.covariance[8]
        self.cov_[2][0] = msg.pose.covariance[12]
        self.cov_[2][1] = msg.pose.covariance[13]
        self.cov_[2][2] = msg.pose.covariance[14]

    def callback_estimator_vel(self, msg):
        self.x_[3] = msg.twist.twist.linear.x
        self.x_[4] = msg.twist.twist.linear.y
        self.x_[5] = msg.twist.twist.linear.z

        self.cov_[3][3] = msg.twist.covariance[0]
        self.cov_[3][4] = msg.twist.covariance[1]
        self.cov_[3][5] = msg.twist.covariance[2]
        self.cov_[4][3] = msg.twist.covariance[6]
        self.cov_[4][4] = msg.twist.covariance[7]
        self.cov_[5][5] = msg.twist.covariance[8]
        self.cov_[5][3] = msg.twist.covariance[12]
        self.cov_[5][4] = msg.twist.covariance[13]
        self.cov_[5][5] = msg.twist.covariance[14]

    def callback_timesync(self, msg):
        self.timestamp_ = msg.timestamp

    def callback_send_odom(self):
        if self.timestamp_ == 0 or self.x_[0] == 0 or self.x_[4] == 0:
            return

        msg = VehicleVisualOdometry()
        msg.timestamp = self.timestamp_
        msg.timestamp_sample = self.timestamp_
        msg.local_frame = VehicleVisualOdometry.LOCAL_FRAME_FRD
        msg.velocity_frame = VehicleVisualOdometry.LOCAL_FRAME_FRD

        # Conversion from ENU to NED
        msg.x = self.x_[1]
        msg.y = self.x_[0]
        msg.z = -self.x_[2]
        msg.q[0] = float('NaN')
        msg.q_offset[0] = float('NaN')

        msg.pose_covariance[0] = self.cov_[1][1]
        msg.pose_covariance[1] = self.cov_[1][0]
        msg.pose_covariance[2] = self.cov_[1][2]
        msg.pose_covariance[6] = self.cov_[0][0]
        msg.pose_covariance[7] = self.cov_[0][2]
        msg.pose_covariance[11] = self.cov_[2][2]
        msg.pose_covariance[15] = float('NaN')

        msg.vx = self.x_[4]
        msg.vy = self.x_[3]
        msg.vz = -self.x_[5]
        msg.rollspeed = float('NaN')
        msg.pitchspeed = float('NaN')
        msg.yawspeed = float('NaN')

        msg.velocity_covariance[0] = self.cov_[4][4]
        msg.velocity_covariance[1] = self.cov_[4][3]
        msg.velocity_covariance[2] = self.cov_[4][5]
        msg.velocity_covariance[6] = self.cov_[3][3]
        msg.velocity_covariance[7] = self.cov_[3][5]
        msg.velocity_covariance[11] = self.cov_[5][5]
        msg.velocity_covariance[15] = float('NaN')

        self.px4_odometry_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbEstimate2Px4()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
