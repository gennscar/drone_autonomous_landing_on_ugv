#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
import tf2_ros as tf
import json

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform


class HandOfGodNav(Node):
    """Script to use the HandOfGod plugin providing the list of positions to reach"""

    def __init__(self, frame_id, target_positions):
        """Initialize Node

        Args:
          frame_id (str): The frame id provided to the plugin
          target_positions (list): A list of positions to be reached by the target
        """
        super().__init__("hand_of_god_nav")

        self.get_logger().info("hand_of_god_nav has started on " + frame_id)

        self.frame_id_ = frame_id
        self.target_positions_ = target_positions
        self.target_counter_ = 0

        self.tf_buffer_ = tf.Buffer()
        self.tf_broadcaster_ = tf.TransformBroadcaster(self)
        self.tf_listener_ = tf.TransformListener(self.tf_buffer_, self)

        self.broadcast_position(self.target_positions_[0]["x"],
                                self.target_positions_[0]["y"],
                                self.target_positions_[0]["z"])

        self.timer = self.create_timer(1, self.timer_callback)

    def broadcast_position(self, x, y, z):
        self.msg_ = TransformStamped()

        self.msg_.header.frame_id = "world"
        self.msg_.child_frame_id = self.frame_id_ + "_desired"

        self.msg_.transform.translation.x = x
        self.msg_.transform.translation.y = y
        self.msg_.transform.translation.z = z

        self.tf_broadcaster_.sendTransform(self.msg_)

    def timer_callback(self):
        if(self.target_counter_ == len(self.target_positions_)):
            return

        # Send desired position
        self.broadcast_position(self.target_positions_[
            self.target_counter_]["x"],
            self.target_positions_[
            self.target_counter_]["y"],
            self.target_positions_[
            self.target_counter_]["z"])

        # Retrieve the actual position
        try:
            trans = self.tf_buffer_.lookup_transform(
                "world", self.frame_id_ + "_actual", Time(seconds=0, nanoseconds=0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # Calculate the error
        actual_pos = np.array([trans.transform.translation.x,
                               trans.transform.translation.y,
                               trans.transform.translation.z])

        desired_pos = np.array([self.msg_.transform.translation.x,
                                self.msg_.transform.translation.y,
                                self.msg_.transform.translation.z])

        error = np.linalg.norm(actual_pos - desired_pos, ord=2)

        # Moving to the next target
        if(error < 0.1):
            self.target_counter_ += 1

        # Logging
        self.get_logger().info(f"""
          Moving:           {self.frame_id_}
          Actual Position:  {actual_pos}
          Desired Position: {desired_pos}
          Error:            {error}
          """)


def test(args=None):
    """main used for test"""

    # Open file with test targets
    fd = open("json/target_test.json", mode="r")

    # Initialize the Node
    rclpy.init(args=args)
    node = HandOfGodNav("uwb_sensor_head", json.load(fd))
    rclpy.spin(node)
    rclpy.shutdown()
