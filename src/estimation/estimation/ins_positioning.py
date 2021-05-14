#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from px4_msgs.msg import SensorCombined
from geometry_msgs.msg import PointStamped


class InsPositioning(Node):
    """
    Node to estimate the position using only the accelerometer
    """

    def __init__(self):
        super().__init__("ins_positioning")

        # Accumulators
        self.velocity_ = np.array([0.0, 0.0, 0.0])
        self.position_ = np.array([0.0, 0.0, 0.0])

        # Timestamp
        self.last_time_ = 0

        # Setting up sensors subscriber
        self.sensor_subscriber_ = self.create_subscription(
            SensorCombined, "/SensorCombined_PubSubTopic", self.callback_sensor_subscriber, 10)

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PointStamped, self.get_namespace() + "/estimated_pos", 10)

        self.get_logger().info("Node has started")

    def callback_sensor_subscriber(self, msg):
        if(self.last_time_ != 0):
            # Dt calculation
            dt = (msg.timestamp - self.last_time_) * 1e-6

            # Removing gravity acceleration
            msg.accelerometer_m_s2[2] += 9.81

            # Discrete integration
            self.velocity_ += msg.accelerometer_m_s2 * dt
            self.position_ += self.velocity_ * dt

            # Filling estimated point message
            est_pos = PointStamped()
            est_pos.header.stamp = self.get_clock().now().to_msg()
            est_pos.header.frame_id = self.get_namespace() + "/estimated_pos"
            est_pos.point.x = self.position_[0]
            est_pos.point.y = self.position_[1]
            est_pos.point.z = self.position_[2]
            self.est_pos_publisher_.publish(est_pos)

        self.last_time_ = msg.timestamp


def main(args=None):
    rclpy.init(args=args)
    node = InsPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
