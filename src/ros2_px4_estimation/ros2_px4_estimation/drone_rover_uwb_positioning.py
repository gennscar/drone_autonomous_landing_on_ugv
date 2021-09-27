#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time, Duration
from ros2_px4_interfaces.msg import UwbSensor, Yaw
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
from px4_msgs.msg import VehicleLocalPosition
import ros2_px4_functions


class UwbPositioning(Node):

    def __init__(self):
        super().__init__("uwb_positioning")

        self.theta_body_wrt_NED = []
        self.anchors_0_ = {}
        self.tag_0_est_pos = [] 
        self.anchors_1_ = {}
        self.tag_1_est_pos = [] 
        self.n_turns_ = 0.0
        self.old_theta_rover_wrt_NED = 0.0

        # Parameters declaration
        self.sensor_id_0_ = self.declare_parameter("sensor_id_0", "tag_0")
        self.sensor_id_1_ = self.declare_parameter("sensor_id_1", "tag_1")
        self.vehicle_namespace = self.declare_parameter("vehicle_namespace", "/drone")
        self.allowed_delay_ns = self.declare_parameter("allowed_delay_ns", 1e2)
        self.max_range = self.declare_parameter("max_range", 50.0)

        # Retrieve parameter values
        self.sensor_id_0_ = self.get_parameter(
            "sensor_id_0").get_parameter_value().string_value
        self.sensor_id_1_ = self.get_parameter(
            "sensor_id_1").get_parameter_value().string_value
        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value
        self.allowed_delay_ns = self.get_parameter(
            "allowed_delay_ns").get_parameter_value().double_value
        self.max_range = self.get_parameter(
            "max_range").get_parameter_value().double_value

        # Namespace check
        if(self.get_namespace() == '/'):
            self.get_logger().error("uwb_positioning need a namespace")
            self.destroy_node()

        # Setting up sensors subscriber for the UWB plugin
        self.tag_0_subscriber = self.create_subscription(
            UwbSensor, "/uwb_sensor_" + self.sensor_id_0_, self.callback_tag_0, 10)
        self.tag_1_subscriber = self.create_subscription(
            UwbSensor, "/uwb_sensor_" + self.sensor_id_1_, self.callback_tag_1, 10)
        self.drone_attitude_subscriber = self.create_subscription(
            VehicleLocalPosition, self.vehicle_namespace + "/VehicleLocalPosition_PubSubTopic", self.callback_drone_attitude, 10)

        # Setting up a publishers to send the estimated position in the NED frame
        self.rot_estimator_topic_name_ = self.get_namespace() + "/estimated_pos"
        self.rot_position_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.rot_estimator_topic_name_, 10)

        self.norot_estimator_topic_name_ = self.get_namespace() + "/norot_pos"
        self.norot_position_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.norot_estimator_topic_name_, 10)

        self.yaw_publisher_topic = self.get_namespace() + "/estimated_yaw"
        self.yaw_publisher = self.create_publisher(Yaw, self.yaw_publisher_topic , 10)

        # Timer
        self.timer_ = self.create_timer(0.1, self.callback_timer)

        self.get_logger().info(f"""Node has started:
                               Sensor ID 0:  {self.sensor_id_0_}
                               Sensor ID 1:  {self.sensor_id_1_}
                              """)

    def callback_drone_attitude(self, msg):

        self.theta_body_wrt_NED = msg.heading

    def callback_tag_0(self, msg):
        # Saving the message in a dict
        if msg.anchor_pose.header.frame_id != "tag_1":
            self.anchors_0_[msg.anchor_pose.header.frame_id] = msg

        # Extract anchor positions and ranges
        i = 0
        anchor_pos = np.empty((len(self.anchors_0_), 3))
        ranges = np.empty(len(self.anchors_0_))

        for _, data in self.anchors_0_.items():
            delta = Time.from_msg(msg.anchor_pose.header.stamp) - \
                Time.from_msg(data.anchor_pose.header.stamp)

            if delta < Duration(nanoseconds=self.allowed_delay_ns) and data.range > 0.0 and data.range <= self.max_range:
                anchor_pos[i, :] = np.array(
                    [data.anchor_pose.pose.position.x, data.anchor_pose.pose.position.y, data.anchor_pose.pose.position.z])
                ranges[i] = data.range
                i = i+1

        N = i
        anchor_pos = anchor_pos[0:N, :]
        ranges = ranges[0:N]

        if N > 2:
            
            tag_0_est_pos, Q_0 = ros2_px4_functions.ls_trilateration(
                anchor_pos, ranges, N)
            #self.get_logger().info(f"{Q_0}")
            if np.linalg.norm(tag_0_est_pos[0:2], ord=2) <= self.max_range:

                self.tag_0_est_pos = tag_0_est_pos

    def callback_tag_1(self, msg):

        # Saving the message in a dict
        if msg.anchor_pose.header.frame_id != "tag_0":
            self.anchors_1_[msg.anchor_pose.header.frame_id] = msg

        # Extract anchor positions and ranges
        i = 0
        anchor_pos = np.empty((len(self.anchors_1_), 3))
        ranges = np.empty(len(self.anchors_1_))

        for _, data in self.anchors_1_.items():
            delta = Time.from_msg(msg.anchor_pose.header.stamp) - \
                Time.from_msg(data.anchor_pose.header.stamp)

            if delta < Duration(nanoseconds=self.allowed_delay_ns) and data.range > 0.0 and data.range <= self.max_range:
                anchor_pos[i, :] = np.array(
                    [data.anchor_pose.pose.position.x, data.anchor_pose.pose.position.y, data.anchor_pose.pose.position.z])
                ranges[i] = data.range
                i = i+1

        N = i
        anchor_pos = anchor_pos[0:N, :]
        ranges = ranges[0:N]

        if N > 2:
            
            tag_1_est_pos, Q_1 = ros2_px4_functions.ls_trilateration(
                anchor_pos, ranges, N)
            self.get_logger().info(f"{Q_1}")
            if np.linalg.norm(tag_1_est_pos[0:2], ord=2) <= self.max_range:

                self.tag_1_est_pos = tag_1_est_pos


    def callback_timer(self):
        
        if self.tag_1_est_pos != [] and self.tag_0_est_pos != [] and self.theta_body_wrt_NED != []:
            
            drone_est_pos_ROV = (self.tag_0_est_pos + self.tag_1_est_pos)/2

            norot_est_pos = PoseWithCovarianceStamped()
            norot_est_pos.header.frame_id = self.get_namespace() + "/norot_pos"
            norot_est_pos.header.stamp = self.get_clock().now().to_msg()
            norot_est_pos.pose.pose.position.x = drone_est_pos_ROV[0]
            norot_est_pos.pose.pose.position.y = drone_est_pos_ROV[1]
            norot_est_pos.pose.pose.position.z = drone_est_pos_ROV[2]
            self.norot_position_publisher.publish(norot_est_pos)

            ######################################

            theta_drone_wrt_rover = - np.arctan2(self.tag_1_est_pos[0] - self.tag_0_est_pos[0],self.tag_0_est_pos[1]-self.tag_1_est_pos[1])
            theta_rover_wrt_body = - np.pi + theta_drone_wrt_rover
            theta_rover_wrt_NED = theta_rover_wrt_body + self.theta_body_wrt_NED

            if theta_rover_wrt_NED - self.old_theta_rover_wrt_NED > 1.5*np.pi:
                self.n_turns_ -= 1
            elif theta_rover_wrt_NED - self.old_theta_rover_wrt_NED < - 1.5*np.pi:
                self.n_turns_ += 1
            self.old_theta_rover_wrt_NED = theta_rover_wrt_NED

            theta_rover_wrt_NED = theta_rover_wrt_NED + self.n_turns_*2*np.pi

            msg = Yaw()
            msg.yaw = theta_rover_wrt_NED
            msg.header.frame_id = self.yaw_publisher_topic
            msg.header.stamp = self.get_clock().now().to_msg()

            self.yaw_publisher.publish(msg)

            ######################################

            rot_rover_to_NED = (R.from_euler(
                'z', theta_rover_wrt_NED, degrees=False))
            drone_est_pos_rot = rot_rover_to_NED.apply(drone_est_pos_ROV)
            drone_est_pos_NED = [drone_est_pos_rot[0], - drone_est_pos_rot[1], drone_est_pos_rot[2]]

            
            rot_est_pos = PoseWithCovarianceStamped()
            rot_est_pos.header.frame_id = self.get_namespace() + "/estimated_pos"
            rot_est_pos.header.stamp = self.get_clock().now().to_msg()
            rot_est_pos.pose.pose.position.x = drone_est_pos_NED[0]
            rot_est_pos.pose.pose.position.y = drone_est_pos_NED[1]
            rot_est_pos.pose.pose.position.z = drone_est_pos_NED[2]
            self.rot_position_publisher.publish(rot_est_pos)



def main(args=None):
    rclpy.init(args=args)
    node = UwbPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
