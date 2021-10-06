# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes

import numpy as np
from ros2_px4_interfaces.msg import Yaw
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry


from random import gauss
from random import seed
# seed random number generator
seed(1)


class GazeboYawNode(Node):

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

        self.yaw_offset = self.declare_parameter("yaw_offset", 0.0)
        self.yaw_std_dev = self.declare_parameter("yaw_std_dev", 0.0)
        self.yaw_publisher_topic = self.declare_parameter("yaw_publisher_topic", "/yaw_sensor/estimated_yaw")

        self.yaw_offset = self.get_parameter(
            "yaw_offset").get_parameter_value().double_value
        self.yaw_std_dev = self.get_parameter(
            "yaw_std_dev").get_parameter_value().double_value
        self.yaw_publisher_topic = self.get_parameter(
            "yaw_publisher_topic").get_parameter_value().string_value

        self.rover_yaw_subscriber = self.create_subscription(Odometry, "rover/odom", self.callback_rover_yaw, 1) 
        self.rover_gazebo_yaw_publisher = self.create_publisher(Yaw, self.yaw_publisher_topic, 10)

    def callback_rover_yaw(self, msg):

        self.rover_quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.rover_rotation = R.from_quat(self.rover_quaternion)

        # Yaw in ENU frame 
        self.rover_yaw_raw = (self.rover_rotation.as_euler(
                'xyz', degrees=True))[2]

        if self.rover_yaw_raw >= -180.0 and self.rover_yaw_raw <= 180.0:
            if self.rover_yaw_raw - self.old_rover_yaw_raw > 300.0:
                self.n_turns_ -= 1
            elif self.rover_yaw_raw - self.old_rover_yaw_raw < - 300.0:
                self.n_turns_ += 1
            self.old_rover_yaw_raw = self.rover_yaw_raw

            if self.yaw_std_dev != 0.0:
                self.rover_yaw = - (self.rover_yaw_raw + self.n_turns_*360.0 + gauss(- self.yaw_offset, self.yaw_std_dev) - 90.0)
            else:
                self.rover_yaw = - (self.rover_yaw_raw + self.n_turns_*360.0 - self.yaw_offset - 90.0)

            msg = Yaw()
            msg.yaw = self.rover_yaw 
            msg.header.frame_id = self.yaw_publisher_topic
            msg.header.stamp = self.get_clock().now().to_msg()

            self.rover_gazebo_yaw_publisher.publish(msg)

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  node = GazeboYawNode()
  
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