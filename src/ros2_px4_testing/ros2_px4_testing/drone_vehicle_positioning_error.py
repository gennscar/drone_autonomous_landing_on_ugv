#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class PositioningError(Node):
    """Node to estimate the position error of estimators"""

    def __init__(self):
        super().__init__("positioning_error")
        self.estimator_topics_ = {}

        self.uwb_position_wrt_world = [0., 0., 0.]
        self.true_pos = [0., 0., 0.]
        self.drone_global_pos = [0., 0., 0.]
        self.chassis_pos = [0., 0., 0.]
        self.chassis_orientation = [0., 0., 0., 1.]

        self.drone_true_position_subscriber = self.create_subscription(
            Odometry, "/uwb_sensor_iris/odom", self.callback_drone_true_position, 10)
        self.chassis_pose_subscriber = self.create_subscription(
            Odometry, "/chassis/odom", self.callback_chassis_true_pose, 10)

        self.ground_truth_publisher = self.create_publisher(
            Point,self.get_namespace() + "/true_pos", 10)

        # Timer to check the creation of new estimators
        self.timer_ = self.create_timer(1, self.timer_callback)

        self.get_logger().info(f"""Node has started""")

    def timer_callback(self):
        """Timer to check the creation of new estimators"""


        for topic_name, _ in self.get_topic_names_and_types():
            if("estimated_pos" in topic_name and topic_name not in self.estimator_topics_.keys()):
                # Creation to a new subscriber and error publisher for the new estimator
                self.create_subscription(
                    PoseWithCovarianceStamped, topic_name, self.callback_uwb_position_wrt_world, 10)
                self.estimator_topics_[topic_name] = self.create_publisher(
                    Float64, topic_name.replace("estimated_pos", "") + "error", 10)

                self.get_logger().info(f"""
                                       Connected to: {topic_name}
                                       """)

    def callback_chassis_true_pose(self, msg):
        self.chassis_pos = [msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.position.z]
        self.chassis_orientation = [msg.pose.pose.orientation.x,
                                     msg.pose.pose.orientation.y,
                                     msg.pose.pose.orientation.z,
                                     msg.pose.pose.orientation.w]
                                     
    def callback_uwb_position_wrt_world(self, msg):

        self.uwb_position_wrt_world = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

        self.true_pos = np.subtract(self.chassis_pos, self.drone_global_pos)

        self.err_vec = np.subtract(self.uwb_position_wrt_world, self.true_pos)
        self.err_xy_vec = np.array([self.err_vec[0], self.err_vec[1]])
        self.err = np.linalg.norm(self.err_xy_vec, ord=2)
        print
        err_ = Float64()
        err_.data = self.err
        
        self.publish_true_pos()

        # Sending error message only if the estimator is valid
        if(msg.header.frame_id in self.estimator_topics_.keys()):
            self.estimator_topics_[msg.header.frame_id].publish(err_)

    def callback_drone_true_position(self, msg):
        self.drone_global_pos = [msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 msg.pose.pose.position.z]
    def publish_true_pos(self):
        
        ground_truth = Point()
        ground_truth.x = self.true_pos[0]
        ground_truth.y = self.true_pos[1]
        ground_truth.z = self.true_pos[2]

        self.ground_truth_publisher.publish(ground_truth)

def main(args=None):
    rclpy.init(args=args)
    node = PositioningError()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
