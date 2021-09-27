#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleGpsPosition
from geometry_msgs.msg import PoseWithCovarianceStamped

import ros2_px4_functions


class GpsPositioning(Node):
    """Node to obtain the position from the GPS sensor"""

    def __init__(self):
        super().__init__("gps_positioning")

        # GPS position
        self.pos_ = np.zeros(3)

        # Setting up PX4 subscribers
        self.gpspos_subscriber_ = self.create_subscription(
            VehicleGpsPosition, "VehicleGpsPosition_PubSubTopic", self.callback_gpspos_subscriber, 10)

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "~/EstimatedPose", 10)

        self.get_logger().info("Node has started")

    def callback_gpspos_subscriber(self, msg):
        lat_rad = math.radians(msg.lat * 1e-7)
        lon_rad = math.radians(msg.lon * 1e-7)
        alt_m = msg.alt * 1e-3

        self.pos_ = ros2_px4_functions.NED_to_ECEF(lat_rad, lon_rad, alt_m)

        # Filling estimated point message
        est_pos = PoseWithCovarianceStamped()
        est_pos.header.stamp = self.get_clock().now().to_msg()
        est_pos.header.frame_id = "~/EstimatedPose"
        est_pos.pose.pose.position.x = self.pos_[0]
        est_pos.pose.pose.position.y = self.pos_[1]
        est_pos.pose.pose.position.z = self.pos_[2]
        self.est_pos_publisher_.publish(est_pos)


def main(args=None):
    rclpy.init(args=args)
    node = GpsPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
