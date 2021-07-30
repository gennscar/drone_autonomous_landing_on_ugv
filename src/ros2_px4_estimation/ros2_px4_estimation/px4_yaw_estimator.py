
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes

import numpy as np
from px4_msgs.msg import VehicleAttitude
from ros2_px4_interfaces.msg import Yaw
from scipy.spatial.transform import Rotation as R
from px4_msgs.msg import Timesync

# generate random Gaussian values
from random import seed
from random import gauss
# seed random number generator
seed(1)


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

        self.vehicle_namespace = self.declare_parameter("vehicle_namespace", '/rover')
        self.noise_ = self.declare_parameter("noise", 1)
        self.timestamp = 0

        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value
        self.noise_ = self.get_parameter(
            "noise").get_parameter_value().integer_value

        self.noise_mean = []
        self.noise_std_dev = 5e-2
        self.noise_mean_freq_ = 5e-8
        self.rot_global2local = R.from_matrix(
            [[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        self.timesync_sub_ = self.create_subscription(
            Timesync, self.vehicle_namespace + "/Timesync_PubSubTopic", self.callback_timesync, 3)
        self.vehicle_orientation_subscriber = self.create_subscription(VehicleAttitude, self.vehicle_namespace + "/VehicleAttitude_PubSubTopic", self.callback_vehicle_orientation, 1) 
        self.estimator_topic_name_ = "/px4_estimator/estimated_yaw"
        self.vehicle_px4_yaw_publisher = self.create_publisher(Yaw, self.estimator_topic_name_, 10)

    def callback_vehicle_orientation(self, msg):
        self.vehicle_orientation = np.array([msg.q[3], msg.q[0], msg.q[1], msg.q[2]])
        self.vehicle_rotation = self.rot_global2local* R.from_quat(self.vehicle_orientation)
        self.vehicle_yaw = - (self.vehicle_rotation.as_euler(
                'xyz', degrees=True))[2]
        if self.noise_==1:
          self.noise_mean = 1*np.sin(2*np.pi*self.noise_mean_freq_*self.timestamp) + 2.

          self.noise_value = gauss(self.noise_mean, self.noise_std_dev)
          self.vehicle_yaw = self.vehicle_yaw + self.noise_value

        msg = Yaw()
        msg.yaw = self.vehicle_yaw 
        msg.header.frame_id = self.estimator_topic_name_
        msg.header.stamp = self.get_clock().now().to_msg()

        self.vehicle_px4_yaw_publisher.publish(msg)

    def callback_timesync(self, msg):
        self.timestamp = msg.timestamp   

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