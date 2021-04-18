#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from  px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

chassis_wrt_anchor0 = np.array([0.94859,0.44998,-0.32457])
p0 = np.array([0.0,0.0,0.0])
p1 = np.array([1.9,0.9,0.0])
p2 = np.array([0.0,0.9,0.0])
p3 = np.array([1.9,0.0,0.2])

a0 = np.concatenate([[1], -2*p0])
a1 = np.concatenate([[1], -2*p1])
a2 = np.concatenate([[1], -2*p2])
a3 = np.concatenate([[1], -2*p3])
A = np.array([a0,a1,a2,a3])
pinvA = np.linalg.pinv(A, rcond=1e-15, hermitian=False)

def trilateration(p0, p1, p2, p3, d0, d1, d2, d3, pinvA):

    b0 = np.array(d0**2 - p0[0]**2 - p0[1]**2 - p0[2]**2)
    b1 = np.array(d1**2 - p1[0]**2 - p1[1]**2 - p1[2]**2)
    b2 = np.array(d2**2 - p2[0]**2 - p2[1]**2 - p2[2]**2)
    b3 = np.array(d3**2 - p3[0]**2 - p3[1]**2 - p3[2]**2)
    b = np.array([b0,b1,b2,b3])

    y = pinvA.dot(b)

    return y


class UwbPubSub(Node):
    def __init__(self):
        super().__init__("uwb_pub_sub")

        self.drone_position_subscriber = self.create_subscription(VehicleLocalPosition,"VehicleLocalPosition_PubSubTopic",self.callback_drone_position,10)
        self.anchor_0_subscriber = self.create_subscription(Odometry,"/anchor_0/odom",self.callback_anchor_0,10)
        self.anchor_1_subscriber = self.create_subscription(Odometry,"/anchor_1/odom",self.callback_anchor_1,10)
        self.anchor_2_subscriber = self.create_subscription(Odometry,"/anchor_2/odom",self.callback_anchor_2,10)
        self.anchor_3_subscriber = self.create_subscription(Odometry,"/anchor_3/odom",self.callback_anchor_3,10)
        self.chassis_subscriber = self.create_subscription(Odometry,"/chassis/odom",self.callback_chassis,10)

        self.get_logger().info("uwb pub sub has started")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_drone_position(self, msg):
        self.drone_local_pos = [msg.x, msg.y, msg.z]
        self.drone_global_pos = [msg.y, msg.x, -msg.z]
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

    def timer_callback(self):


        self.d0 = np.linalg.norm(self.anchor_0_pos, ord = 2)
        self.d1 = np.linalg.norm(self.anchor_1_pos, ord = 2)
        self.d2 = np.linalg.norm(self.anchor_2_pos, ord = 2)
        self.d3 = np.linalg.norm(self.anchor_3_pos, ord = 2)

        self.y = trilateration(p0, p1, p2, p3, self.d0, self.d1, self.d2, self.d3, pinvA)
        self.rel_pos_anchor0 = self.y[1:]
        self.rot_world_to_anchor0 = R.from_quat(self.anchor_0_orientation)

        self.rel_pos_chassis = np.subtract(self.rel_pos_anchor0, chassis_wrt_anchor0)
        self.rel_pos_world_frame = - self.rot_world_to_anchor0.apply(self.rel_pos_chassis)

        self.err_vec = np.subtract(self.rel_pos_world_frame ,self.chassis_pos)
        self.err = np.linalg.norm(self.err_vec, ord = 2)

        #self.get_logger().info(f"""
        #trilateration:{self.rel_pos_world_frame}
        #true: {self.chassis_pos}""")
        #self.get_logger().info(f"{self.err_vec}")
        self.get_logger().info(f"{self.err}")







def main(args = None):

    rclpy.init(args = args)
    node = UwbPubSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()