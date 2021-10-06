#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleGpsPosition, VehicleLocalPosition
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

import ros2_px4_functions

QUEUE_SIZE = 10


class GpsPositioning(Node):
    """Node to obtain the data from the GPS sensor in a local ENU frame"""

    def __init__(self):
        super().__init__("GpsPositioning")

        # Reference of ENU frame
        self.reference_ = np.zeros(3)
        self.is_ref_valid = False

        # Setting up PX4 subscribers
        self.gpspos_subscriber_ = self.create_subscription(
            VehicleGpsPosition, "VehicleGpsPosition_PubSubTopic",
            self.callback_gpspos_subscriber, QUEUE_SIZE
        )
        self.gpspos_subscriber_ = self.create_subscription(
            VehicleLocalPosition, "VehicleLocalPosition_PubSubTopic",
            self.callback_local_subscriber, QUEUE_SIZE
        )

        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "~/EstimatedPosition", QUEUE_SIZE)
        self.est_vel_publisher_ = self.create_publisher(
            TwistWithCovarianceStamped, "~/EstimatedVelocity", QUEUE_SIZE)

        self.get_logger().info("Node has started")

    def callback_local_subscriber(self, msg):
        """This callback ask to PX4 the position of the ENU reference frame

        Args:
            msg (from px4_msgs.msg.VehicleLocalPosition): The PX4 messages
            containing info about the local frame positioning
        """

        self.reference_ = np.array([
            math.radians(msg.ref_lat),
            math.radians(msg.ref_lon),
            msg.ref_alt
        ])
        self.is_ref_valid = True

    def callback_gpspos_subscriber(self, msg):
        """Retrieve GPS data from sensors and convert them in the local ENU
        reference frame

        Args:
            msg (from px4_msgs.msg.VehicleGpsPosition): PX4 messages forwarding
            GPS data
        """

        # Check if the reference is valid
        if not self.is_ref_valid:
            self.get_logger().warn("Waiting for local reference frame definition")

        # Converting into the right measurements units
        coordinates = np.array([
            math.radians(msg.lat*1e-7),
            math.radians(msg.lon*1e-7),
            msg.alt*1e-3
        ])

        # Converting from WGS84 to local ENU frame
        self.pos_ = ros2_px4_functions.WGS84_to_ENU(
            coordinates, self.reference_)

        # Filling estimated position message
        est_pos = PoseWithCovarianceStamped()
        est_pos.header.stamp = self.get_clock().now().to_msg()
        est_pos.header.frame_id = "GpsPositioning"

        est_pos.pose.pose.position.x = self.pos_[0]
        est_pos.pose.pose.position.y = self.pos_[1]
        est_pos.pose.pose.position.z = self.pos_[2]

        est_pos.pose.covariance[0] = msg.eph
        est_pos.pose.covariance[7] = msg.eph
        est_pos.pose.covariance[15] = msg.epv

        self.est_pos_publisher_.publish(est_pos)

        # Filling estimated velocity message
        est_vel = TwistWithCovarianceStamped()
        est_vel.header.stamp = self.get_clock().now().to_msg()
        est_vel.header.frame_id = "GpsPositioning"

        est_vel.twist.twist.linear.x = msg.vel_e_m_s
        est_vel.twist.twist.linear.y = msg.vel_n_m_s
        est_vel.twist.twist.linear.z = -msg.vel_d_m_s

        est_vel.twist.covariance[0] = msg.s_variance_m_s
        est_vel.twist.covariance[7] = msg.s_variance_m_s
        est_vel.twist.covariance[15] = msg.s_variance_m_s

        self.est_vel_publisher_.publish(est_vel)


def main(args=None):
    rclpy.init(args=args)
    node = GpsPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
