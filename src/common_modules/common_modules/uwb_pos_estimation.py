#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from gazebo_msgs.msg import UwbSensor
from geometry_msgs.msg import Pose

# seed the pseudorandom number generator
from random import seed
from random import random
# seed random number generator
seed(1)

from common_modules import standard_trilateration
from common_modules import gauss_newton_trilateration

STD_TRILATERATION = True


class UwbPubSub(Node):
    def __init__(self):
        super().__init__("uwb_pub_sub")

        self.dict = {}
        self.drone_global_pos = [0.0, 0.0, 0.0]
        self.num_anchors = 0
        self.y = [random(),random(),random()]

        self.drone_position_subscriber = self.create_subscription(VehicleLocalPosition,"VehicleLocalPosition_PubSubTopic",self.callback_drone_position,10)
        self.sensor_subscriber = self.create_subscription(UwbSensor,"uwb_sensor_0",self.callback_drone_position,10)

        self.target_coordinates_publisher = self.create_publisher(Pose,"target_coordinates",10)
        
        self.get_logger().info("uwb pub sub has started")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_drone_position(self, msg):
        self.drone_local_pos = [msg.x, msg.y, msg.z]
        self.drone_global_pos = [msg.y+1.0, msg.x+1.0, -msg.z]

    def callback_sensor_subscriber(self, msg):
        self.dict[msg.anchor_id] = msg


    def send_target_coordinates(self):
        msg = Pose()
        msg.position.x = self.y[0]
        msg.position.y = self.y[1]
        msg.position.z = self.y[2]

        self.target_coordinates_publisher.publish(msg)

    def timer_callback(self):
        
        self.num_anchors = len(self.dict.values())
        for i in range(self.num_anchors):
            self.anchor[i] = list(self.dict.values())[i]
            self.d[i] = self.anchor[i].range
            self.p[i] = [self.anchor[i].anchor_pos.x, self.anchor[i].anchor_pos.y, self.anchor[i].anchor_pos.z]
            print(d[i])
        
        if self.num_anchors >= 4:
            pass
        """

            if STD_TRILATERATION == True:
                self.y = standard_trilateration.trilateration(self.d0, self.d1, self.d2, self.d3)
                self.y = self.y[1:]
            else:
                self.y = gauss_newton_trilateration.trilateration(self.y, self.d0, self.d1, self.d2, self.d3)

            self.get_logger().info(f"computed pos: {self.y}")

            self.get_logger().info(f"true pos: {self.drone_global_pos}")

            self.err_vec = np.subtract(self.y , self.drone_global_pos)
            self.err = np.linalg.norm(self.err_vec, ord = 2)

            #self.send_target_coordinates()
            """

            #self.get_logger().info(f"""
            #trilateration:{self.y}
            #true: {self.chassis_pos}""")
            #self.get_logger().info(f"{self.err_vec}")
            #self.get_logger().info(f"{self.y}")

            #self.get_logger().info(f"{self.err}")




def main(args = None):

    rclpy.init(args = args)
    node = UwbPubSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()