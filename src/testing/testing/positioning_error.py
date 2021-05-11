#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class PositioningError(Node):
    """Node to estimate the position error of estimators"""

    def __init__(self):
        super().__init__("positioning_error")

        self.sensor_real_pos_ = []
        self.estimator_topics_ = {}

        self.sensor_real_subscriber_ = self.create_subscription(
            Odometry, "/uwb_sensor_iris/odom", self.callback_real_subscriber, 10)

        self.timer_ = self.create_timer(1, self.timer_callback)

        self.get_logger().info("positioning_error has started")

    def timer_callback(self):
        for topic_name, topic_type in self.get_topic_names_and_types():
            if("estimated_pos" in topic_name and "mse" not in topic_name and topic_name not in self.estimator_topics_.keys()):
                self.create_subscription(
                    PointStamped, topic_name, self.callback_sensor_subscriber, 10)
                self.estimator_topics_[topic_name] = self.create_publisher(
                    Float64, topic_name + "/mse", 10)

    def callback_sensor_subscriber(self, msg):
        error = Float64()
        sensor_est_pos = np.array([msg.point.x, msg.point.y, msg.point.z])

        error.data = np.linalg.norm(
            sensor_est_pos - self.sensor_real_pos_, ord=2)

        if(msg.header.frame_id in self.estimator_topics_.keys()):
            self.estimator_topics_[msg.header.frame_id].publish(error)

    def callback_real_subscriber(self, msg):
        self.sensor_real_pos_ = np.array([msg.pose.pose.position.x,
                                         msg.pose.pose.position.y,
                                         msg.pose.pose.position.z])


def main(args=None):
    rclpy.init(args=args)
    node = PositioningError()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
