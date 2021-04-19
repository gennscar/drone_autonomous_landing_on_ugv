#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class TargetPublisher(Node):

    def __init__(self):
        super().__init__("target_position_publisher")

        self.pub = self.create_publisher(Pose,"target_coordinates",10)

        self.timer = self.create_timer(0.1, self.send_target_coordinates)
        self.get_logger().info("Target publisher has been started")

    def send_target_coordinates(self):
        msg = Pose()
        msg.position.x = 0.0
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args = args)
    node = TargetPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
