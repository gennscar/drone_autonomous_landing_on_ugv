#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# seed the pseudorandom number generator
from random import seed
from random import gauss
# seed random number generator
seed(1)

dt = 0.1
noise_dt = 2
t1 = 200 #100 #200
t2 = t1 + 30

lin_vel = 1.0
ang_vel = 0.5

curvature = True 
noise = False


class VehicleController(Node):
    def __init__(self):
        super().__init__("vehicle_controller")
        
        self.vehicle_namespace = self.declare_parameter("vehicle_namespace", '/rover')

        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value

        self.i = 0
        self.lin_noise_val = 0.0
        self.ang_noise_val = 0.0

        self.rover_controller_publisher=self.create_publisher(Twist, self.vehicle_namespace + "/cmd_vel",10)
        self.timer = self.create_timer(dt, self.timer_callback)
        self.noise_timer = self.create_timer(noise_dt, self.noise_timer_callback)

    def timer_callback(self):
        self.i += 1
        if self.i >= t1 and self.i < t2:
            self.publish_rover_command_2()
        elif self.i == t2:
            self.i = 0
            self.publish_rover_command_1()
        else: 
            self.publish_rover_command_1()

    def noise_timer_callback(self):
        self.lin_noise_val = gauss(0.2,0.2)
        self.ang_noise_val = gauss(0,0.05)

    def publish_rover_command_1(self):
        msg = Twist()
        if noise:
            msg.linear.x = - lin_vel + self.lin_noise_val
            msg.angular.z = 0.0 + self.ang_noise_val

        else:
            msg.linear.x = - lin_vel
            msg.angular.z = 0.0

        self.rover_controller_publisher.publish(msg)

    def publish_rover_command_2(self):
        msg = Twist()
        msg.linear.x = - lin_vel
        if curvature:
            msg.angular.z = ang_vel
        else:
            msg.angular.z = 0.0
        self.rover_controller_publisher.publish(msg)



def main(args=None):

    rclpy.init(args=args)
    node = VehicleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
