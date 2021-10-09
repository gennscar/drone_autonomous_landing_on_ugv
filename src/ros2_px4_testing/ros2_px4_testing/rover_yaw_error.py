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

        self.estimator_topics_ = {}
        self.true_yaw = 0.0
        self.true_yaw_raw = 0.0
        self.old_true_yaw_raw = 0.0
        self.n_turns_ = 0.0

        self.vehicle_namespace = self.declare_parameter("vehicle_namespace", '/rover')

        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value

        # Sensor subscriber to the real position
        self.true_yaw_subscriber = self.create_subscription(Odometry, self.vehicle_namespace + "/odom", self.callback_true_yaw, 1) 

        self.true_yaw_publisher = self.create_publisher(Yaw, self.vehicle_namespace + "/true_yaw",10)
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
        error.data = float(self.true_yaw - msg.yaw)**2

        # Sending error message only if the estimat)or is valid
        if(msg.header.frame_id in self.estimator_topics_.keys()):
            self.estimator_topics_[msg.header.frame_id].publish(error)


    def callback_true_yaw(self, msg):

        self.true_quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.true_rotation = R.from_quat(self.true_quaternion)
        
        # Yaw in ENU frame
        self.true_yaw_raw = (self.true_rotation.as_euler(
                'xyz', degrees=True))[2] 

        if self.true_yaw_raw >= -180.0 and self.true_yaw_raw <= 180.0:
            if self.true_yaw_raw - self.old_true_yaw_raw > 300.0:
                self.n_turns_ -= 1
            elif self.true_yaw_raw - self.old_true_yaw_raw < - 300.0:
                self.n_turns_ += 1
            self.old_true_yaw_raw = self.true_yaw_raw

            self.true_yaw = - (self.true_yaw_raw + self.n_turns_*360.0 - 90)

            msg = Yaw()
            msg.yaw = self.true_yaw 
            msg.header.frame_id = self.vehicle_namespace + "/true_yaw"
            msg.header.stamp = self.get_clock().now().to_msg()

            self.true_yaw_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YawError()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
