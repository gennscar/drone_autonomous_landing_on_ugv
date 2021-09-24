#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

from px4_msgs.msg import DistanceSensor
#from sensor_msgs.msg import Range

class Px4Positioning(Node):

    def __init__(self):
        super().__init__("range_sensor_positioning")

        self.vehicle_namespace = self.declare_parameter("vehicle_namespace", "/drone")
        self.declare_parameter("rng_sensor_max_height", 10.0)
        self.declare_parameter("rng_sensor_min_height", 0.2)

        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value
        self.rng_sensor_max_height_ = self.get_parameter('rng_sensor_max_height').get_parameter_value().double_value
        self.rng_sensor_min_height_ = self.get_parameter('rng_sensor_min_height').get_parameter_value().double_value

        #self.range_sensor_subscriber = self.create_subscription(
        #    Range, self.vehicle_namespace + "/DistanceSensor_PubSubTopic", self.callback_range_sensor, 10)
        self.range_sensor_subscriber = self.create_subscription(
            DistanceSensor, self.vehicle_namespace + "/DistanceSensor_PubSubTopic", self.callback_range_sensor, 10)
        
        # Setting up position publisher
        self.est_pos_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, self.get_namespace() + "/estimated_pos", 10)

        self.get_logger().info("Node has started")

    def callback_range_sensor(self, msg):
        # Filling estimated point message

        # distance = msg.range
        distance = msg.current_distance
        if  distance <= self.rng_sensor_max_height_ and \
            distance >= self.rng_sensor_min_height_:
            
            est_pos = PoseWithCovarianceStamped()
            est_pos.header.stamp = self.get_clock().now().to_msg()
            est_pos.header.frame_id = self.get_namespace() + "/estimated_pos"
            est_pos.pose.pose.position.z = distance
            self.est_pos_publisher_.publish(est_pos)


def main(args=None):
    rclpy.init(args=args)
    node = Px4Positioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
