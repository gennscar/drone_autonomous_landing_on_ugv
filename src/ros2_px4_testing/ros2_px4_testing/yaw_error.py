#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from ros2_px4_interfaces.msg import Yaw
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R

class YawError(Node):
    """Node to estimate the position error of estimators"""

    def __init__(self):
        super().__init__("yaw_error")

        self.sensor_real_pos_ = []
        self.estimator_topics_ = {}
        self.true_yaw = []

        # Sensor subscriber to the real position
        self.true_yaw_subscriber = self.create_subscription(Odometry, "/rover_uwb/odom", self.callback_true_yaw, 1) 

        # Timer to check the creation of new estimators
        self.timer_ = self.create_timer(1, self.timer_callback)

        self.get_logger().info(f"""Node has started""")

    def timer_callback(self):
        """Timer to check the creation of new estimators"""

        for topic_name, _ in self.get_topic_names_and_types():
            if("estimated_yaw" in topic_name and topic_name not in self.estimator_topics_.keys()):
                # Creation to a new subscriber and error publisher for the new estimator
                self.create_subscription(
                    Yaw, topic_name, self.callback_sensor_subscriber, 10)
                self.estimator_topics_[topic_name] = self.create_publisher(
                    Float64, topic_name.replace("estimated_yaw", "") + "yaw_error", 10)

                self.get_logger().info(f"""
                                       Connected to: {topic_name}
                                       """)

    def callback_sensor_subscriber(self, msg):


        # Filling error message
        error = Float64()
        error.data = self.true_yaw - msg.yaw

        # Sending error message only if the estimator is valid
        if(msg.header.frame_id in self.estimator_topics_.keys()):
            self.estimator_topics_[msg.header.frame_id].publish(error)


    def callback_true_yaw(self, msg):

        self.true_attitude = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.true_rotation = R.from_quat(self.true_attitude)
        self.true_yaw = (self.true_rotation.as_euler(
                'xyz', degrees=True))[2] + 90 


def main(args=None):
    rclpy.init(args=args)
    node = YawError()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
