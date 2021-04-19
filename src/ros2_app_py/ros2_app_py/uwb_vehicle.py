#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from  px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from common_modules.standard_trilateration import trilateration

chassis_wrt_anchor0 = np.array([0.94859,0.44998,-0.32457])

class UwbPubSub(Node):
    def __init__(self):
        super().__init__("uwb_pub_sub")

        self.rel_pos_world_frame = [0.0, 0.0, 0.0]
        self.drone_global_pos = [0.0, 0.0, 0.0]
        self.chassis_pos = [0.0, 0.0, 0.0]
        self.anchor_0_pos = [0.0, 0.0, 0.0]
        self.anchor_1_pos = [0.0, 0.0, 0.0]
        self.anchor_2_pos = [0.0, 0.0, 0.0]
        self.anchor_3_pos = [0.0, 0.0, 0.0]
        self.anchor_0_orientation = [0.0, 0.0, 0.0, 1.0]
        self.anchor_1_orientation = [0.0, 0.0, 0.0, 1.0]
        self.anchor_2_orientation = [0.0, 0.0, 0.0, 1.0]
        self.anchor_3_orientation = [0.0, 0.0, 0.0, 1.0]

        self.drone_position_subscriber = self.create_subscription(VehicleLocalPosition,"VehicleLocalPosition_PubSubTopic",self.callback_drone_position,10)
        self.anchor_0_subscriber = self.create_subscription(Odometry,"/anchor_0/odom",self.callback_anchor_0,10)
        self.anchor_1_subscriber = self.create_subscription(Odometry,"/anchor_1/odom",self.callback_anchor_1,10)
        self.anchor_2_subscriber = self.create_subscription(Odometry,"/anchor_2/odom",self.callback_anchor_2,10)
        self.anchor_3_subscriber = self.create_subscription(Odometry,"/anchor_3/odom",self.callback_anchor_3,10)
        self.chassis_subscriber = self.create_subscription(Odometry,"/chassis/odom",self.callback_chassis,10)

        self.target_coordinates_publisher = self.create_publisher(Pose,"target_coordinates",10)
        
        self.get_logger().info("uwb pub sub has started")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_drone_position(self, msg):
        self.drone_local_pos = [msg.x, msg.y, msg.z]
        self.drone_global_pos = [msg.y+1.0, msg.x+1.0, -msg.z]
        self.drone_local_vel = [msg.vx, msg.vy, msg.vz]
        self.drone_global_vel = [msg.vy, msg.vx, -msg.vz]


    def callback_chassis(self, msg):
        self.chassis_pos = [msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z]

    def callback_anchor_0(self, msg):
        self.anchor_0_pos = [msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z]
        self.anchor_0_orientation = [msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w]


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

    def send_target_coordinates(self):
        msg = Pose()
        msg.position.x = self.rel_pos_world_frame[0]
        msg.position.y = self.rel_pos_world_frame[1]
        msg.position.z = self.rel_pos_world_frame[2]

        self.target_coordinates_publisher.publish(msg)

    def timer_callback(self):

        self.r0 = np.subtract(self.anchor_0_pos, self.drone_global_pos)
        self.r1 = np.subtract(self.anchor_1_pos, self.drone_global_pos)
        self.r2 = np.subtract(self.anchor_2_pos, self.drone_global_pos)
        self.r3 = np.subtract(self.anchor_3_pos, self.drone_global_pos)

        self.d0 = np.linalg.norm(self.r0, ord = 2)
        self.d1 = np.linalg.norm(self.r1, ord = 2)
        self.d2 = np.linalg.norm(self.r2, ord = 2)
        self.d3 = np.linalg.norm(self.r3, ord = 2)

        self.y = trilateration(self.d0, self.d1, self.d2, self.d3)
        
        self.rel_pos_anchor0 = self.y[1:]
        self.rot_world_to_anchor0 = R.from_quat(self.anchor_0_orientation)

        self.rel_pos_chassis = np.subtract(self.rel_pos_anchor0, chassis_wrt_anchor0)
        self.rel_pos_world_frame = - self.rot_world_to_anchor0.apply(self.rel_pos_chassis)
        self.get_logger().info(f"computed pos: {self.rel_pos_world_frame}")

        
        self.true_pos = np.subtract(self.chassis_pos, self.drone_global_pos)
        self.get_logger().info(f"true pos: {self.true_pos}")

        self.err_vec = np.subtract(self.rel_pos_world_frame , self.true_pos)
        self.err = np.linalg.norm(self.err_vec, ord = 2)

        #self.send_target_coordinates()

        #self.get_logger().info(f"""
        #trilateration:{self.rel_pos_world_frame}
        #true: {self.chassis_pos}""")
        #self.get_logger().info(f"{self.err_vec}")
        #self.get_logger().info(f"{self.rel_pos_world_frame}")

        #self.get_logger().info(f"{self.err}")




def main(args = None):

    rclpy.init(args = args)
    node = UwbPubSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()