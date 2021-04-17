#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from  px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry


class UwbPubSub(Node):
    def __init__(self):
        super().__init__("uwb_pub_sub")

        #self.anchor_0_mob_frame = [0,0,0]
        #self.anchor_1_mob_frame = [0,0,0]
        #self.anchor_2_mob_frame = [0,0,0]
        #self.anchor_3_mob_frame = [0,0,0]



        self.drone_position_subscriber = self.create_subscription(VehicleLocalPosition,"VehicleLocalPosition_PubSubTopic",self.callback_drone_position,10)
        self.fixed_anchor_subscriber = self.create_subscription(Odometry,"/fixed_anchor/odom",self.callback_fixed_anchor,10)
        self.anchor_0_subscriber = self.create_subscription(Odometry,"/anchor_0/odom",self.callback_anchor_0,10)
        self.anchor_1_subscriber = self.create_subscription(Odometry,"/anchor_1/odom",self.callback_anchor_1,10)
        self.anchor_2_subscriber = self.create_subscription(Odometry,"/anchor_2/odom",self.callback_anchor_2,10)
        self.anchor_3_subscriber = self.create_subscription(Odometry,"/anchor_3/odom",self.callback_anchor_3,10)

        self.get_logger().info("uwb pub sub has started")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_drone_position(self, msg):
        self.drone_local_pos = [msg.x, msg.y, msg.z]
        self.drone_local_vel = [msg.vx, msg.vy, msg.vz]

    def callback_fixed_anchor(self, msg):
        self.fixed_anchor_pos = [msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z]

    def callback_anchor_0(self, msg):
        self.anchor_0_pos = [msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z]

    def callback_anchor_1(self, msg):
        self.anchor_1_pos = [msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z]

    def callback_anchor_2(self, msg):
        self.anchor_2_pos = [msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z]

    def callback_anchor_3(self, msg):
        self.anchor_3_pos = [msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z]

    def timer_callback(self):

        #self.get_logger().info(f"""
        #Fixed Anchor: {self.fixed_anchor_pos}
        #Anchor 0: {self.anchor_0_pos}
        #Anchor 1: {self.anchor_1_pos}
        #Anchor 2: {self.anchor_2_pos}
        #Anchor 3: {self.anchor_3_pos}""")

        self.r0 = np.subtract(self.anchor_0_pos,self.fixed_anchor_pos)
        self.r1 = np.subtract(self.anchor_1_pos,self.fixed_anchor_pos)
        self.r2 = np.subtract(self.anchor_2_pos,self.fixed_anchor_pos)
        self.r3 = np.subtract(self.anchor_3_pos,self.fixed_anchor_pos)

        self.d0 = np.linalg.norm(self.r0, ord = 2)
        self.d1 = np.linalg.norm(self.r1, ord = 2)
        self.d2 = np.linalg.norm(self.r2, ord = 2)
        self.d3 = np.linalg.norm(self.r3, ord = 2)

        self.get_logger().info(f"""
        d0: {self.d0}
        d1: {self.d1}
        d2: {self.d2}
        d3: {self.d3}""")






def main(args = None):

    rclpy.init(args = args)
    node = UwbPubSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()