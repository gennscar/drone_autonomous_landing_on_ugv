#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from gazebo_msgs.msg import UwbSensor
from nav_msgs.msg import Odometry

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
        
        self.sensor_true_pos = []
        self.distances = []
        self.positions = []
        self.dict = {}
        #self.drone_global_pos = [0.0, 0.0, 0.0]
        self.num_anchors = 0
        self.y = [random(),random(),random()]

        self.drone_position_subscriber = self.create_subscription(VehicleLocalPosition,"VehicleLocalPosition_PubSubTopic",self.callback_drone_position,10)
        self.sensor_subscriber = self.create_subscription(UwbSensor,"uwb_sensor_0",self.callback_sensor_subscriber,10)
        self.sensor_true_subscriber = self.create_subscription(Odometry,"/uwb_sensor_true/odom",self.callback_sensor_true_subscriber,10)

        self.target_coordinates_publisher = self.create_publisher(Pose,"target_coordinates",10)
        
        self.get_logger().info("uwb pub sub has started")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_drone_position(self, msg):
        self.drone_local_pos = [msg.x, msg.y, msg.z]
        self.drone_global_pos = [msg.y+1.0, msg.x+1.0, -msg.z]

    def callback_sensor_subscriber(self, msg):
        self.dict[msg.anchor_id] = msg

    def callback_sensor_true_subscriber(self, msg):
        self.sensor_true_pos = [msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z]

    def send_target_coordinates(self):
        msg = Pose()
        msg.position.x = self.y[0]
        msg.position.y = self.y[1]
        msg.position.z = self.y[2]

        self.target_coordinates_publisher.publish(msg)

    def timer_callback(self):
        
        self.num_anchors = len(self.dict.values())
        if self.num_anchors != 0:
            self.distances = []
            self.positions = []
            for i in range(self.num_anchors):
                self.distances.append( list(self.dict.values())[i].range )
                self.positions.append( [list(self.dict.values())[i].anchor_pos.x, list(self.dict.values())[i].anchor_pos.y, list(self.dict.values())[i].anchor_pos.z] )
    
    
        if self.num_anchors >= 4:
            if STD_TRILATERATION == True:
                self.y = standard_trilateration.trilateration(self.distances[0], self.distances[1], self.distances[2], self.distances[3], self.positions[0], self.positions[1], self.positions[2], self.positions[3])
                self.y = self.y[1:]
            else:
                self.y = gauss_newton_trilateration.trilateration(self.y, self.d0, self.d1, self.d2, self.d3)

            self.err_vec = np.subtract(self.y , self.sensor_true_pos)
            self.err = np.linalg.norm(self.err_vec, ord = 2)
            self.get_logger().info(f"""
            Computed pos: {self.y}
            True pos: {self.sensor_true_pos}
            Error: {self.err}""")
            
    
            #self.get_logger().info(f"true pos: {self.drone_global_pos}")

            #self.err_vec = np.subtract(self.y , self.drone_global_pos)
            #self.err = np.linalg.norm(self.err_vec, ord = 2)

            #self.send_target_coordinates()
            

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