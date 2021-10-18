#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from ros2_px4_interfaces.msg import Yaw
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float64

class YawError(Node):
    """Node to estimate the position error of estimators"""

    def __init__(self):
        super().__init__("yaw_error")

        self.estimator_topics_ = {}

        self.vehicle_namespace = self.declare_parameter("vehicle_namespace", '/X500_0')

        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value

        # Sensor subscriber to the real position
        self.rover_yaw_subscriber = self.create_subscription(Yaw, "/yaw_sensor/estimated_yaw", self.callback_rover_yaw, 1) 
        self.drone_yaw_subscriber = self.create_subscription(VehicleLocalPosition, self.vehicle_namespace + "/VehicleLocalPosition_PubSubTopic", self.callback_drone_yaw, 1) 
        self.yaw_error_publisher = self.create_publisher(Float64, "/yaw_error", 1)

        self.get_logger().info(f"""Node has started""")

        self.rover_yaw = []
        self.drone_yaw = []
        self.n_turns_ = 0
        self.old_drone_yaw_raw = 0

    def callback_rover_yaw(self, msg):
        
        self.rover_yaw = msg.yaw
        self.get_logger().info(f"rover {self.rover_yaw}")
        self.get_logger().info(f"drone {self.drone_yaw}")

        # Filling error message
        if self.drone_yaw is not []:
            error = Float64()
            error.data = float(self.drone_yaw - self.rover_yaw)
            self.yaw_error_publisher.publish(error)
            #self.get_logger().info(f"{error}")


    def callback_drone_yaw(self, msg):

        self.drone_yaw_raw = np.degrees(msg.heading)        

        
        if self.drone_yaw_raw >= -180.0 and self.drone_yaw_raw <= 180.0:
            if self.drone_yaw_raw - self.old_drone_yaw_raw > 300.0:
                self.n_turns_ -= 1
            elif self.drone_yaw_raw - self.old_drone_yaw_raw < - 300.0:
                self.n_turns_ += 1
            self.old_drone_yaw_raw = self.drone_yaw_raw

            self.drone_yaw = (self.drone_yaw_raw + self.n_turns_*360.0)
        #self.get_logger().info(f"{self.drone_yaw}")
        
def main(args=None):
    rclpy.init(args=args)
    node = YawError()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()
