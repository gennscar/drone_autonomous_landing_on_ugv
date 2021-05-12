#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

# seed the pseudorandom number generator
from random import seed
from random import gauss
# seed random number generator
seed(1)

dt = 0.1
t1 = 100 #200
t2 = t1 + 30
lin_vel = 2.5
curvature = False
ang_vel = 0.5

noise = False
noise_dt = 2

class VehicleController(Node):
    def __init__(self):
        super().__init__("vehicle_controller")

<<<<<<< HEAD
        self.i = 0
        self.lin_noise_val = 0.0
        self.ang_noise_val = 0.0
=======
        self.waypoint = 0
        self.waypoint_list = [[20, 0], [-20, 0]]
        self.vehicle_pos = [0, 0]
        self.curr_waypoint = self.waypoint_list[self.waypoint]

        self.vehicle_controller_publisher = self.create_publisher(
            Twist, "/rover_pic4ser/cmd_vel", 10)
        self.vehicle_state_subscriber = self.create_subscription(
            Odometry, "/rover_pic4ser/odom", self.vehicle_state_callback, 10)
>>>>>>> 0665663ad900e2f30202ae499d89853eb126a5e7

        self.vehicle_controller_publisher=self.create_publisher(Twist,"/rover_uwb/cmd_vel",10)
        self.timer = self.create_timer(dt, self.timer_callback)
        self.noise_timer = self.create_timer(noise_dt, self.noise_timer_callback)

    def timer_callback(self):
<<<<<<< HEAD
        self.i += 1
        if self.i >= t1 and self.i < t2:
            self.publish_vehicle_command_2()
        elif self.i == t2:
            self.i = 0
            self.publish_vehicle_command_1()
        else: 
            self.publish_vehicle_command_1()

    def noise_timer_callback(self):
        self.lin_noise_val = gauss(0.5,0.5)
        self.ang_noise_val = gauss(0,0.05)

    def publish_vehicle_command_1(self):
        msg = Twist()
        if noise:
            msg.linear.x = - lin_vel + self.lin_noise_val
            msg.angular.z = 0.0 + self.ang_noise_val

        else:
            msg.linear.x = - lin_vel
            msg.angular.z = 0.0

        self.vehicle_controller_publisher.publish(msg)

    def publish_vehicle_command_2(self):
=======
        self.e = np.subtract(self.curr_waypoint, self.vehicle_pos)
        self.norm_e = np.linalg.norm(self.e, ord=2)

        if self.norm_e < 1:
            self.waypoint += 1
            if self.waypoint == 2:
                self.waypoint = 0
            self.curr_waypoint = self.waypoint_list[self.waypoint]

        self.publish_vehicle_command()

    def publish_vehicle_command(self):
>>>>>>> 0665663ad900e2f30202ae499d89853eb126a5e7
        msg = Twist()
        msg.linear.x = - lin_vel
        if curvature:
            msg.angular.z = ang_vel
        else:
            msg.angular.z = 0.0
        self.vehicle_controller_publisher.publish(msg)

<<<<<<< HEAD
=======
    def vehicle_state_callback(self, msg):
        self.vehicle_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
>>>>>>> 0665663ad900e2f30202ae499d89853eb126a5e7


def main(args=None):

    rclpy.init(args=args)
    node = VehicleController()
    rclpy.spin(node)
    rclpy.shutdown()


<<<<<<< HEAD

=======
>>>>>>> 0665663ad900e2f30202ae499d89853eb126a5e7
if __name__ == "main":
    main()
