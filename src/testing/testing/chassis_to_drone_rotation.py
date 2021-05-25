#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float64



class UwbPubSub(Node):
    def __init__(self):
        super().__init__("uwb_pub_sub")

        self.uwb_position_wrt_world = [0.0, 0.0, 0.0]
        self.uwb_position_wrt_chassis = [0.0, 0.0, 0.0]

        self.drone_global_pos = [0.0, 0.0, 0.0]
        self.chassis_pos = [0.0, 0.0, 0.0]
        self.chassis_orientation = [0.0, 0.0, 0.0, 1.0]

        # Parameters declaration
        self.estimation_mode = self.declare_parameter("estimation_mode", "GN_10iter_uwb_estimator")

        # Retrieve parameter values
        self.estimation_mode = self.get_parameter(
            "estimation_mode").get_parameter_value().string_value
        self.estimation_topic = '/' + self.estimation_mode + '/estimated_pos'

        self.drone_position_subscriber = self.create_subscription(
            Odometry, "/uwb_sensor_iris/odom", self.callback_drone_position, 10)
        self.chassis_position_subscriber = self.create_subscription(
            Odometry, "/chassis/odom", self.callback_chassis_position, 10)
        self.uwb_position_wrt_chassis_position_subscriber = self.create_subscription(
            PointStamped, self.estimation_topic, self.callback_uwb_position_wrt_chassis, 10)
        self.target_coordinates_publisher = self.create_publisher(
            Point, "target_coordinates", 10)
        self.uwb_error_publisher = self.create_publisher(
            Float64, "uwb_error", 10)
        self.get_logger().info("uwb pub sub has started")
        self.timer = self.create_timer(0.1, self.timer_callback)


    def callback_chassis_position(self, msg):
        self.chassis_pos = [msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.position.z]
        self.chassis_orientation = [msg.pose.pose.orientation.x,
                                     msg.pose.pose.orientation.y,
                                     msg.pose.pose.orientation.z,
                                     msg.pose.pose.orientation.w]
    def callback_uwb_position_wrt_chassis(self, msg):
        self.uwb_position_wrt_chassis = [msg.point.x, msg.point.y, msg.point.z]

    def callback_drone_position(self, msg):
        self.drone_global_pos = [msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 msg.pose.pose.position.z]

    def publish_target_coordinates(self):
        msg = Point()
        msg.x = self.uwb_position_wrt_world[0]
        msg.y = self.uwb_position_wrt_world[1]
        msg.z = self.uwb_position_wrt_world[2]

        self.target_coordinates_publisher.publish(msg)

    def timer_callback(self):

        self.rot_world_to_chassis = R.from_quat(self.chassis_orientation)

        self.uwb_position_wrt_world = - self.rot_world_to_chassis.apply(self.uwb_position_wrt_chassis)

        self.true_pos = np.subtract(self.chassis_pos, self.drone_global_pos)
        self.err_vec = np.subtract(self.uwb_position_wrt_world, self.true_pos)
        self.err = np.linalg.norm(self.err_vec, ord=2)

        self.publish_target_coordinates()
        err_ = Float64()
        err_.data = self.err
        self.uwb_error_publisher.publish(err_)

def main(args=None):

    rclpy.init(args=args)
    node = UwbPubSub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()

