#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleGpsPosition
from nav_msgs.msg import Odometry
from ros2_px4_interfaces.srv import GpsEnable

import ros2_px4_functions

QUEUE_SIZE = 10


class GpsPositioning(Node):
    """Node to obtain the data from the GPS sensor in a local ENU frame"""

    def __init__(self):
        super().__init__("GpsPositioning")

        # Enable bit
        self.enable_ = False

        # Reference of ENU frame
        self.reference_ = np.zeros(3)
        self.ref_counter_ = 0

        # Setting up PX4 subscribers
        self.gpspos_subscriber_ = self.create_subscription(
            VehicleGpsPosition, "VehicleGpsPosition_PubSubTopic",
            self.callback_gpspos_subscriber, QUEUE_SIZE
        )

        # Setting up position publisher
        self.odometry_publisher_ = self.create_publisher(
            Odometry, "~/Odometry", QUEUE_SIZE)

        # Services
        self.control_mode_service = self.create_service(
            GpsEnable, "GpsEnable_Service", self.callback_service)

        self.get_logger().info("Node has started")

    def callback_service(self, request, response):
        """This callback answer to the request to enable/disable the GPS

        Args:
        request (dict): structure containing the request received.
        response (dict): structure containing the response to send.
        """

        # Setting up mode
        self.enable_ = request.enable

        response.message = "Gps is " + \
            ("enabled" if self.enable_ else "disabled")
        self.get_logger().info(f"{response.message}")

        return response

    def callback_gpspos_subscriber(self, msg):
        """Retrieve GPS data from sensors and convert them in the local ENU
        reference frame

        Args:
            msg (from px4_msgs.msg.VehicleGpsPosition): PX4 messages forwarding
            GPS data
        """

        # Converting into the right measurements units
        coordinates = np.array([
            math.radians(msg.lat*1e-7),
            math.radians(msg.lon*1e-7),
            msg.alt*1e-3
        ])

        # Check if the reference is valid
        if self.ref_counter_ < 10:
            self.reference_ += (coordinates - self.reference_) / \
                (self.ref_counter_ + 1)
            self.ref_counter_ += 1
            return

        # Converting from WGS84 to local ENU frame
        self.pos_ = ros2_px4_functions.WGS84_to_ENU(
            coordinates, self.reference_)

        # Send GPS only if enabled
        if not self.enable_:
            return

        # Filling estimated position message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "GpsPositioning"

        odom.pose.pose.position.x = self.pos_[0]
        odom.pose.pose.position.y = self.pos_[1]
        odom.pose.pose.position.z = self.pos_[2]

        odom.pose.covariance[0] = msg.eph**2
        odom.pose.covariance[7] = msg.eph**2
        odom.pose.covariance[15] = msg.epv**2

        odom.twist.twist.linear.x = msg.vel_e_m_s
        odom.twist.twist.linear.y = msg.vel_n_m_s
        odom.twist.twist.linear.z = -msg.vel_d_m_s

        odom.twist.covariance[0] = msg.s_variance_m_s**2
        odom.twist.covariance[7] = msg.s_variance_m_s**2
        odom.twist.covariance[15] = msg.s_variance_m_s**2

        self.odometry_publisher_.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GpsPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
