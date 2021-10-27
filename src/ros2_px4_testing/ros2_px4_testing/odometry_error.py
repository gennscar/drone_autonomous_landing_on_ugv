#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleLocalPositionGroundtruth
from ros2_px4_interfaces.msg import Error


class OdometryError(Node):
    """Node to estimate the position error of estimators"""

    def __init__(self):
        super().__init__("odometry_error")

        self.sensor_real_pos_ = []

        # Sensor subscriber to the real position
        self.sensor_real_subscriber_ = self.create_subscription(
            VehicleLocalPositionGroundtruth, "VehicleLocalPositionGroundtruth_PubSubTopic", self.callback_real_subscriber, 10)

        # Sensor subscriber to the estimated position
        self.sensor_est_subscriber_ = self.create_subscription(
            VehicleLocalPosition, "VehicleLocalPosition_PubSubTopic", self.callback_est_subscriber, 10)

        # Error
        self.sensor_err_publisher_ = self.create_publisher(
            Error, "LocalPositionError", 10
        )

        self.get_logger().info(f"""Node has started""")

    def callback_est_subscriber(self, msg):
        """
        Retriving the estimated/sensor position and evaluating mean square error

        Args:
            msg (PoseWithCovarianceStamped): Estimated pose
        """

        # Retriving the estimated position
        sensor_est_pos = np.array([
            msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz,
        ])

        # Filling error message
        error = Error()
        error.header.stamp = self.get_clock().now().to_msg()
        error.header.frame_id = "This nuts"

        if(self.sensor_real_pos_ != []):
            diff = sensor_est_pos - self.sensor_real_pos_

            error.rmse = np.linalg.norm(diff)
            error.mse = error.rmse**2

            """
            P = np.array([
                [
                    msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[2]
                ],
                [
                    msg.pose.covariance[6], msg.pose.covariance[7], msg.pose.covariance[8]
                ],
                [
                    msg.pose.covariance[12], msg.pose.covariance[13], msg.pose.covariance[14]
                ]
            ])

            try:
                Pinv = np.linalg.inv(P)
            except:
                pass
            else:
                error.nees = np.matmul(
                    np.matmul(np.transpose(diff), Pinv), diff)
            """

        # Sending error message only if the estimator is valid
        self.sensor_err_publisher_.publish(error)

    def callback_real_subscriber(self, msg):
        """
        Retriving the true position

        Args:
            msg (Odometry): True position
        """
        self.sensor_real_pos_ = np.array([
            msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz,
        ])


def main(args=None):
    rclpy.init(args=args)
    node = OdometryError()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
