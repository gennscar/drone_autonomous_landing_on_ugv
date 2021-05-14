#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from custom_interfaces.msg import Error


class PositioningError(Node):
    """Node to estimate the position error of estimators"""

    def __init__(self):
        super().__init__("positioning_error")

        self.sensor_real_pos_ = []
        self.estimator_topics_ = {}

        self.sensor_real_subscriber_ = self.create_subscription(
            Odometry, "/uwb_sensor_iris/odom", self.callback_real_subscriber, 10)

        self.timer_ = self.create_timer(1, self.timer_callback)

        self.get_logger().info(f"""
                                positioning_error has started
                               """)

    def timer_callback(self):
        for topic_name, _ in self.get_topic_names_and_types():
            if("estimated_pos" in topic_name and topic_name not in self.estimator_topics_.keys()):
                self.create_subscription(
                    PointStamped, topic_name, self.callback_sensor_subscriber, 10)
                self.estimator_topics_[topic_name] = self.create_publisher(
                    Error, topic_name.replace("estimated_pos", "") + "mse", 10)

    def callback_sensor_subscriber(self, msg):
        error = Error()
        sensor_est_pos = np.array([msg.point.x, msg.point.y, msg.point.z])

        error.header.stamp = self.get_clock().now().to_msg()
        error.header.frame_id = msg.header.frame_id
        if(self.sensor_real_pos_ == []):
            error.current = float('NaN')
        else:
            error.current = np.linalg.norm(
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
