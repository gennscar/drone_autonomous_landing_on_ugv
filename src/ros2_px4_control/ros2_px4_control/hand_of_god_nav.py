#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros as tf

from geometry_msgs.msg import TransformStamped


class HandOfGodNav(Node):
    """Node to use the HandOfGod plugin providing the positions to reach"""

    def __init__(self):
        super().__init__("hand_of_god_nav")

        # Parameters declaration
        self.declare_parameter("frame_id", "")
        self.declare_parameter("reference_id", "world")
        self.declare_parameter("target_position", [0.0, 0.0, 0.0])

        # Setting up tf2 broadcaster for communicating the targets to the plugin
        self.tf_broadcaster_ = tf.TransformBroadcaster(self)

        # Broadcasting the desired position every 1 sec
        self.timer = self.create_timer(1, self.broadcast_position)

        self.get_logger().info("hand_of_god_nav node has started")

    def broadcast_position(self):
        # Parameter getters
        frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value

        reference_id = self.get_parameter(
            'reference_id').get_parameter_value().string_value

        target_position = self.get_parameter(
            'target_position').get_parameter_value().double_array_value

        if(len(target_position) != 3):
            self.get_logger().warn(f"""
                                   <target_position> parameter not valid" {target_position}
                                   """)
            return

        # Sending the desired position
        self.msg_ = TransformStamped()

        self.msg_.header.frame_id = reference_id
        self.msg_.child_frame_id = frame_id + "_desired"

        self.msg_.transform.translation.x = target_position[0]
        self.msg_.transform.translation.y = target_position[1]
        self.msg_.transform.translation.z = target_position[2]

        self.tf_broadcaster_.sendTransform(self.msg_)


def main(args=None):
    rclpy.init(args=args)
    node = HandOfGodNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
