#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from  px4_msgs.msg import VehicleLocalPosition

class LocalPositionSubscriberNode(Node):
    def __init__(self):
        super().__init__("local_position_subscriber")
        self.gps_subscriber = self.create_subscription(VehicleLocalPosition,"VehicleLocalPosition_PubSubTopic",self.callback_local_position,10)
        self.get_logger().info("Local position subscriber has started")
    def callback_local_position(self, msg):
        self.get_logger().info(f"\nx: {msg.x}\ny: {msg.y}\nz: {msg.z}\nvx: {msg.vx}\nvy: {msg.vy}\nvz: {msg.vz}")

def main(args = None):

    rclpy.init(args = args)
    node = LocalPositionSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()