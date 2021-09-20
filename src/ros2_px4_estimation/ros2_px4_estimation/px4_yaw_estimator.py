# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes

import numpy as np
from px4_msgs.msg import VehicleAttitude
from ros2_px4_interfaces.msg import Yaw
from scipy.spatial.transform import Rotation as R

class PX4YawNode(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('node')

        self.rot_NED_2_ENU = R.from_matrix(
            [[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        self.old_rover_yaw = 0.0
        self.old_rover_yaw_raw = 0.0
        self.rover_yaw = 0.0
        self.n_turns_ = 0.0

        self.vehicle_namespace = self.declare_parameter("vehicle_namespace", '/rover')

        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value

        self.rover_px4_yaw_subscriber = self.create_subscription(VehicleAttitude, self.vehicle_namespace + "/VehicleAttitude_PubSubTopic", self.callback_rover_px4_yaw, 1) 

        self.estimator_topic_name_ = "/px4_estimator/estimated_yaw"
        self.rover_px4_yaw_publisher = self.create_publisher(Yaw, self.estimator_topic_name_, 10)

    def callback_rover_px4_yaw(self, msg):

        self.rover_quaternion = np.array([msg.q[3], msg.q[0], msg.q[1], msg.q[2]])
        self.rover_rotation = self.rot_NED_2_ENU* R.from_quat(self.rover_quaternion)

        # Yaw in ENU frame 
        self.rover_yaw_raw = - (self.rover_rotation.as_euler(
                'xyz', degrees=True))[2]

        if self.rover_yaw_raw >= -180.0 and self.rover_yaw_raw <= 180.0:
            if self.rover_yaw_raw - self.old_rover_yaw_raw > 300.0:
                self.n_turns_ -= 1
            elif self.rover_yaw_raw - self.old_rover_yaw_raw < - 300.0:
                self.n_turns_ += 1
            self.old_rover_yaw_raw = self.rover_yaw_raw

            self.rover_yaw = self.rover_yaw_raw + self.n_turns_*360.0

            msg = Yaw()
            msg.yaw = self.rover_yaw 
            msg.header.frame_id = self.estimator_topic_name_
            msg.header.stamp = self.get_clock().now().to_msg()

            self.rover_px4_yaw_publisher.publish(msg)

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  node = PX4YawNode()
  
  # Spin the node so the callback function is called.
  rclpy.spin(node)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  node.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()