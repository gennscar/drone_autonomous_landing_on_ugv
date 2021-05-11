#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math


class VehicleController(Node):
    def __init__(self):
        super().__init__("vehicle_controller")

        self.waypoint = 0
        self.waypoint_list = [[20,0],[-20,0]]
        self.vehicle_pos = [0,0]
        self.curr_waypoint = self.waypoint_list[self.waypoint]

        self.vehicle_controller_publisher=self.create_publisher(Twist,"/rover_pic4ser/cmd_vel",10)
        self.vehicle_state_subscriber=self.create_subscription(Odometry,"/rover_pic4ser/odom",self.vehicle_state_callback,10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.e = np.subtract(self.curr_waypoint, self.vehicle_pos)
        self.norm_e = np.linalg.norm(self.e, ord=2)
            
        if  self.norm_e < 1:
            self.waypoint += 1
            if self.waypoint == 2:
                self.waypoint = 0
            self.curr_waypoint = self.waypoint_list[self.waypoint]


        self.publish_vehicle_command()


    def publish_vehicle_command(self):
        msg = Twist()
        if self.waypoint == 0:
            msg.linear.x = 2.5
        else:
            msg.linear.x = -2.5

        self.vehicle_controller_publisher.publish(msg)

    def vehicle_state_callback(self,msg):
        self.vehicle_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]

def main(args = None):

    rclpy.init(args = args)
    node = VehicleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()