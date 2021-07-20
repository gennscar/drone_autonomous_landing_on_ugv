#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitude
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32


class OffsetError(Node):


    def __init__(self):
        super().__init__("uwb_positioning")

        self.yaw_drone_2 = []
        self.yaw_drone_1 = []
        
        # Parameters declaration
        self.drone_1_ = self.declare_parameter("drone_1", "/X500_0")
        self.drone_2_ = self.declare_parameter("drone_2", "/X500_1")

        # Retrieve parameter values
        self.drone_1_ = self.get_parameter(
            "drone_1").get_parameter_value().string_value
        self.drone_2_ = self.get_parameter(
            "drone_2").get_parameter_value().string_value

        # Setting subscribers
        self.drone_1_attitude = self.create_subscription(
            VehicleAttitude, self.drone_1_ + "/VehicleAttitude_PubSubTopic", self.callback_drone_1_attitude, 10)
        self.drone_2_attitude = self.create_subscription(
            VehicleAttitude, self.drone_2_ + "/VehicleAttitude_PubSubTopic", self.callback_drone_2_attitude, 10)

        # Setting timer
        self.timer_ = self.create_timer(0.1, self.callback_offset_error)

        # Setting publisher
        self.offset_yaw_publisher = self.create_publisher(Float32, "offset_yaw",10)

        self.get_logger().info("The node has started")

    def callback_drone_1_attitude(self, msg):
        self.attitude_drone_1 = np.array([msg.q[3], msg.q[0], msg.q[1], msg.q[2]])
        self.rotation_drone_1 = R.from_quat(self.attitude_drone_1)
        self.yaw_drone_1 = - (self.rotation_drone_1.as_euler(
                'xyz', degrees=True))[2] + 180

    def callback_drone_2_attitude(self, msg):
        self.attitude_drone_2 = np.array([msg.q[3], msg.q[0], msg.q[1], msg.q[2]])
        self.rotation_drone_2 = R.from_quat(self.attitude_drone_2)
        self.yaw_drone_2 = - (self.rotation_drone_2.as_euler(
                'xyz', degrees=True))[2] + 180


    def callback_offset_error(self):

        if self.yaw_drone_1 == [] or self.yaw_drone_2 == []:
            return
        else:
            self.offset_yaw_ = self.yaw_drone_1 - self.yaw_drone_2
            self.msg_ = Float32()
            self.msg_.data =self.offset_yaw_
            self.offset_yaw_publisher.publish(self.msg_)
            self.get_logger().info(f"""OFFSET: {self.offset_yaw_}
            yaw1: {self.yaw_drone_1}
            yaw2: {self.yaw_drone_2}""")


        



def main(args=None):
    rclpy.init(args=args)
    node = OffsetError()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
