#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros as tf

from geometry_msgs.msg import TransformStamped

class HandOfGodNav(Node):
    def __init__(self):
        super().__init__("hand_of_god_nav")
        self.get_logger().info("hand_of_god_nav has started")

        self.transform_broadcaster_ =  tf.TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = TransformStamped()

        x = float(input('Set X:'))

        msg.header.frame_id = "world"
        msg.child_frame_id = "uwb_sensor_head_desired"
        msg.transform.translation.x = x
        msg.transform.translation.y = x
        msg.transform.translation.z = x 
        
        self.transform_broadcaster_.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandOfGodNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
