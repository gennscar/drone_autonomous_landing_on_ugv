#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PointStamped


class Px4Positioning(Node):
    """
    Node to obtain the position estimated by PX4
    """

    def __init__(self):
        super().__init__("px4_positioning")

        # Setting up PX4 subscribers
        self.px4pos_subscriber_ = self.create_subscription(
            VehicleLocalPosition, "/VehicleLocalPosition_PubSubTopic", self.callback_px4pos_subscriber, 10)

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PointStamped, self.get_namespace() + "/estimated_pos", 10)

        self.get_logger().info("Node has started")

    def callback_px4pos_subscriber(self, msg):

        # Filling estimated point message
        est_pos = PointStamped()
        est_pos.header.stamp = self.get_clock().now().to_msg()
        est_pos.header.frame_id = self.get_namespace() + "/estimated_pos"
        est_pos.point.x = msg.x + 1.0
        est_pos.point.y = msg.y + 1.0
        est_pos.point.z = -msg.z
        self.est_pos_publisher_.publish(est_pos)

        self.last_time_ = msg.timestamp


def main(args=None):
    rclpy.init(args=args)
    node = Px4Positioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
