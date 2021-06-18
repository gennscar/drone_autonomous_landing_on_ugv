#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from px4_msgs.msg import VehicleAttitude
from ros2_px4_interfaces.msg import UwbSensor
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from scipy.spatial.transform import Rotation as R

import ros2_px4_functions


class UwbPositioning(Node):
    """
    Node to estimate the position using only the UWB

    Params:
      sensor_id (str): The anchor ID of the UWB sensor that need to be tracked
        is the <anchor_id> param provided @ the UWB gazebo plugin

      method (str): Can be "LS" or "GN":
        "LS" is the Least-Square method
        "GN" is for the Gauss-Newton method

      iterations (int): Numbers of iterations, valid only with the GN method
    """

    def __init__(self):
        super().__init__("uwb_positioning")

        self.anchors_ = {}
        self.sensor_est_pos_ = [0.01, 0.01, 0.01]  # @todo Need to be random

        self.px4_offset_rotation = R.from_matrix(
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.rot_global2local = R.from_matrix(
            [[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        self.offset_yaw = 0.

        # Parameters declaration
        self.sensor_id_ = self.declare_parameter("sensor_id", "0")
        self.method_ = self.declare_parameter("method", "LS")
        self.iterations_ = self.declare_parameter("iterations", 1)

        # Retrieve parameter values
        self.sensor_id_ = self.get_parameter(
            "sensor_id").get_parameter_value().string_value
        self.method_ = self.get_parameter(
            "method").get_parameter_value().string_value
        self.iterations_ = self.get_parameter(
            "iterations").get_parameter_value().integer_value

        # Namespace check
        if(self.get_namespace() == '/'):
            self.get_logger().error("uwb_positioning need a namespace")
            self.destroy_node()

        # Setting up sensors subscriber for the UWB plugin
        self.sensor_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor_" + self.sensor_id_, self.callback_sensor_subscriber, 10)
        self.px4_attitude_subscriber = self.create_subscription(
            VehicleAttitude, "/rover/VehicleAttitude_PubSubTopic", self.callback_px4_attitude, 10)

        # Setting up a publishers to send the estimated position
        self.estimator_topic_name_ = self.get_namespace() + "/estimated_pos"
        self.rotated_position_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.estimator_topic_name_, 10)
        self.norot_estimator_topic_name_ = self.get_namespace() + "/norot_pos"
        self.norot_position_publisher = self.create_publisher(
            Point, self.norot_estimator_topic_name_, 10)

        self.get_logger().info(f"""Node has started:
                               Sensor ID:  {self.sensor_id_}
                               Method:     {self.method_}
                               Iterations  {self.iterations_}
                              """)

    def callback_px4_attitude(self, msg):
        self.px4_attitude = np.array([msg.q[3], msg.q[0], msg.q[1], msg.q[2]])
        self.px4_attitude_rotation = self.rot_global2local * \
            R.from_quat(self.px4_attitude)
        self.px4_attitude_yaw = - \
            (self.px4_attitude_rotation.as_euler(
                'xyz', degrees=True))[2] + self.offset_yaw
        self.px4_offset_rotation = (R.from_euler(
            'z', self.px4_attitude_yaw, degrees=True))

    def callback_sensor_subscriber(self, msg):
        """
        Receive a pseudorange from the UWB sensor

        Args:
            msg (UwbSensor): The message received by the plugin
        """

        # Saving the message in a dict
        self.anchors_[msg.anchor_id] = msg

        # Extract anchor positions and ranges
        i = 0
        anchor_pos = np.empty((len(self.anchors_), 3))
        ranges = np.empty(len(self.anchors_))

        for _, data in self.anchors_.items():
            if msg.timestamp - data.timestamp < 0.01:
                anchor_pos[i, :] = np.array(
                    [data.anchor_pos.x, data.anchor_pos.y, data.anchor_pos.z])
                ranges[i] = data.range
                i = i+1

        N = i
        anchor_pos = anchor_pos[0:N, :]
        ranges = ranges[0:N]

        # Only if trilateration is possible
        if N > 3:
            # Perform Least-Square
            if(self.method_ == "LS"):
                self.sensor_est_pos_ = ros2_px4_functions.ls_trilateration(
                    anchor_pos, ranges, N)

            # Perform Gauss-Newton
            if(self.method_ == "GN"):
                # Perform N iterations of GN
                for _ in range(self.iterations_):
                    self.sensor_est_pos_ = ros2_px4_functions.gauss_newton_trilateration(
                        self.sensor_est_pos_, anchor_pos, ranges)

            # Sending the estimated position and the name of the node that generated it
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.estimator_topic_name_
            msg.header.stamp = self.get_clock().now().to_msg()
            self.rotated_sensor_est_pos_ = - \
                self.px4_offset_rotation.apply(self.sensor_est_pos_)

            msg.pose.pose.position.x = self.rotated_sensor_est_pos_[0]
            msg.pose.pose.position.y = self.rotated_sensor_est_pos_[1]
            msg.pose.pose.position.z = self.rotated_sensor_est_pos_[2]

            self.rotated_position_publisher.publish(msg)

            msg_2 = Point()
            msg_2.x = self.sensor_est_pos_[0]
            msg_2.y = self.sensor_est_pos_[1]
            msg_2.z = self.sensor_est_pos_[2]
            self.norot_position_publisher.publish(msg_2)


def main(args=None):
    rclpy.init(args=args)
    node = UwbPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
