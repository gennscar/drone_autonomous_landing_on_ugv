
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np


class VideoStreamerNode(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('node')
        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
    
        # Create the subscriber
        self.subscription1 = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
                
        self.subscription1 # prevent unused variable warning
        
       
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
    

    def camera_callback(self, data):
        """
        Callback function.
        """    
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(data)
        frame_rgb = frame[:, :, ::-1].copy()    
  
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image message        
        # Display image
        cv2.imshow("Video", frame_rgb)
        cv2.waitKey(1)    
        
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  node = VideoStreamerNode()
  
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