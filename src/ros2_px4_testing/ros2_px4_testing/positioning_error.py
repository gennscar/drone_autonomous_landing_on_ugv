#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from ros2_px4_interfaces.msg import Error


class PositioningError(Node):
    """Node to estimate the position error of estimators"""

    def __init__(self):
        super().__init__("positioning_error")

        self.sensor_real_pos_ = []
        self.estimator_topics_ = {}

        # Sensor subscriber to the real position
        self.sensor_real_subscriber_ = self.create_subscription(
            Odometry, "/uwb_sensor_iris/odom", self.callback_real_subscriber, 10)

        # Timer to check the creation of new estimators
        self.timer_ = self.create_timer(1, self.timer_callback)

        self.get_logger().info(f"""Node has started""")

    def timer_callback(self):
        """Timer to check the creation of new estimators"""

        for topic_name, _ in self.get_topic_names_and_types():
            if("estimated_pos" in topic_name and topic_name not in self.estimator_topics_.keys()):
                # Creation to a new subscriber and error publisher for the new estimator
                self.create_subscription(
                    PoseWithCovarianceStamped, topic_name, self.callback_sensor_subscriber, 10)
                self.estimator_topics_[topic_name] = self.create_publisher(
                    Error, topic_name.replace("estimated_pos", "") + "error", 10)

                self.get_logger().info(f"""
                                       Connected to: {topic_name}
                                       """)

    def callback_sensor_subscriber(self, msg):
        """
        Retriving the estimated/sensor position and evaluating mean square error

        Args:
            msg (PoseWithCovarianceStamped): Estimated pose
        """

        # Retriving the estimated position
        sensor_est_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # Filling error message
        error = Error()
        error.header.stamp = self.get_clock().now().to_msg()
        error.header.frame_id = msg.header.frame_id

        if(self.sensor_real_pos_ != []):
            diff = sensor_est_pos - self.sensor_real_pos_

            error.rmse = np.linalg.norm(diff)
            error.mse = error.rmse**2

            P = np.array([
                [
                    msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[2]
                ],
                [
                    msg.pose.covariance[3], msg.pose.covariance[4], msg.pose.covariance[5]
                ],
                [
                    msg.pose.covariance[6], msg.pose.covariance[7], msg.pose.covariance[8]
                ]
            ])

            try:
                Pinv = np.linalg.inv(P)
            except:
                pass
            else:
                error.nees = np.matmul(
                    np.matmul(np.transpose(diff), Pinv), diff)

        # Sending error message only if the estimator is valid
        if(msg.header.frame_id in self.estimator_topics_.keys()):
            self.estimator_topics_[msg.header.frame_id].publish(error)

    def callback_real_subscriber(self, msg):
        """
        Retriving the true position

        Args:
            msg (Odometry): True position
        """
        self.sensor_real_pos_ = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])


def main(args=None):
    rclpy.init(args=args)
    node = PositioningError()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
