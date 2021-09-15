#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time, Duration
from ros2_px4_interfaces.msg import UwbSensor
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from scipy.spatial.transform import Rotation as R
from ros2_px4_interfaces.msg import Yaw
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
        self.sensor_est_pos_ = [0.01, 0.01, 0.01] 

        self.rover_rotation = R.from_matrix(
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        
        # Parameters declaration
        self.sensor_id_ = self.declare_parameter("sensor_id", "Iris")
        self.method_ = self.declare_parameter("method", "LS")
        self.iterations_ = self.declare_parameter("iterations", 1)
        self.vehicle_namespace = self.declare_parameter("vehicle_namespace", '/rover')
        self.yaw_estimator = self.declare_parameter("yaw_estimator", '/px4_estimator')

        # Retrieve parameter values
        self.sensor_id_ = self.get_parameter(
            "sensor_id").get_parameter_value().string_value
        self.method_ = self.get_parameter(
            "method").get_parameter_value().string_value
        self.iterations_ = self.get_parameter(
            "iterations").get_parameter_value().integer_value
        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value
        self.yaw_estimator = self.get_parameter(
            "yaw_estimator").get_parameter_value().string_value

        # Namespace check
        if(self.get_namespace() == '/'):
            self.get_logger().error("uwb_positioning need a namespace")
            self.destroy_node()

        # Setting up sensors subscriber for the UWB plugin
        self.sensor_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor_" + self.sensor_id_, self.callback_sensor_subscriber, 10)

        # Setting up sensor subscriber for vehicle yaw
        self.rover_attitude_subscriber = self.create_subscription(Yaw, self.yaw_estimator + "/estimated_yaw", self.callback_rover_attitude, 10)
        
        # Setting up a publishers to send the estimated position in the NED frame
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

    def callback_rover_attitude(self, msg):

        self.rover_rotation = (R.from_euler(
            'z', msg.yaw, degrees=True))

    def callback_sensor_subscriber(self, msg):
        """
        Receive a pseudorange from the UWB sensor

        Args:
            msg (UwbSensor): The message received by the plugin
        """

        # Saving the message in a dict
        self.anchors_[msg.anchor_pose.header.frame_id] = msg

        # Extract anchor positions and ranges
        i = 0
        anchor_pos = np.empty((len(self.anchors_), 3))
        ranges = np.empty(len(self.anchors_))

        for _, data in self.anchors_.items():
            delta = Time.from_msg(msg.anchor_pose.header.stamp) - \
                Time.from_msg(data.anchor_pose.header.stamp)

            if delta < Duration(nanoseconds=1e2):
                anchor_pos[i, :] = np.array(
                    [data.anchor_pose.pose.position.x, data.anchor_pose.pose.position.y, data.anchor_pose.pose.position.z])
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

            # Rotating the vector into the ENU rover frame
            self.rotated_sensor_est_pos_ = - self.rover_rotation.apply(self.sensor_est_pos_)

            msg.pose.pose.position.x = self.rotated_sensor_est_pos_[0]
            msg.pose.pose.position.y = self.rotated_sensor_est_pos_[1]
            msg.pose.pose.position.z = self.rotated_sensor_est_pos_[2]

            self.rotated_position_publisher.publish(msg)

            msg = Point()
            msg.x = self.sensor_est_pos_[0]
            msg.y = self.sensor_est_pos_[1]
            msg.z = self.sensor_est_pos_[2]
            self.norot_position_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
