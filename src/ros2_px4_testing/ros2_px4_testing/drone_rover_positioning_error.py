#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class PositioningError(Node):
    """Node to estimate the position norm_xy_positioning_erroror of estimators"""

    def __init__(self):
        super().__init__("positioning_norm_xy_positioning_erroror")

        self.sensor_id_ = self.declare_parameter("sensor_id", "Iris")
        self.vehicle_namespace = self.declare_parameter("vehicle_namespace", '/rover')

        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value
        self.sensor_id_ = self.get_parameter(
            "sensor_id").get_parameter_value().string_value

        self.estimator_topics_ = {}

        self.estimated_position = [0., 0., 0.]
        self.true_position = [0., 0., 0.]

        self.drone_true_position = [0., 0., 0.]
        self.rover_true_position = [0., 0., 0.]
        self.rover_true_orientation = [0., 0., 0., 1.]

        self.drone_true_pose_subscriber = self.create_subscription(
            Odometry, "/uwb_sensor_" + self.sensor_id_ + "/odom", self.callback_drone_true_pose, 10)
        self.rover_true_pose_subscriber = self.create_subscription(
            Odometry, self.vehicle_namespace + "/odom", self.callback_rover_true_pose, 10)

        self.true_position_publisher = self.create_publisher(
            Point,self.get_namespace() + "/true_position", 10)

        # Timer to check the creation of new estimators
        self.timer_ = self.create_timer(1, self.timer_callback)

        self.get_logger().info(f"""Node has started""")

    def timer_callback(self):
        """Timer to check the creation of new estimators"""
        for topic_name, _ in self.get_topic_names_and_types():
            if("estimated_pos" in topic_name and topic_name not in self.estimator_topics_.keys()):
                # Creation to a new subscriber and norm_xy_positioning_erroror publisher for the new estimator
                self.create_subscription(
                    PoseWithCovarianceStamped, topic_name, self.callback_position_estimator, 10)
                self.estimator_topics_[topic_name] = self.create_publisher(
                    Float64, topic_name.replace("estimated_pos", "") + "error", 10)

                self.get_logger().info(f"""
                                       Connected to: {topic_name}
                                       """)

    def callback_rover_true_pose(self, msg):
        self.rover_true_position = [msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.position.z]
        self.rover_true_orientation = [msg.pose.pose.orientation.x,
                                     msg.pose.pose.orientation.y,
                                     msg.pose.pose.orientation.z,
                                     msg.pose.pose.orientation.w]

    def callback_drone_true_pose(self, msg):
        self.drone_true_position = [msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 msg.pose.pose.position.z]

    def callback_position_estimator(self, msg):
        
        # Rover ENU frame
        self.estimated_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.true_position = np.subtract(self.drone_true_position, self.rover_true_position)
        self.positioning_error_vector = np.subtract(self.estimated_position, self.true_position)
        self.positioning_error_xy = np.array([self.positioning_error_vector[0], self.positioning_error_vector[1]])
        self.norm_xy_positioning_error = np.linalg.norm(self.positioning_error_xy, ord=2)

        norm_xy_positioning_error_ = Float64()
        norm_xy_positioning_error_.data = self.norm_xy_positioning_error
        
        self.publish_true_position()

        # Sending norm_xy_positioning_erroror message only if the estimator is valid
        if(msg.header.frame_id in self.estimator_topics_.keys()):
            self.estimator_topics_[msg.header.frame_id].publish(norm_xy_positioning_error_)

    def publish_true_position(self):
        
        true_position = Point()
        true_position.x = self.true_position[0]
        true_position.y = self.true_position[1]
        true_position.z = self.true_position[2]

        self.true_position_publisher.publish(true_position)

def main(args=None):
    rclpy.init(args=args)
    node = PositioningError()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()
