
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R

camera_params = [613.2378540039062, 613.2378540039062, 325.7916564941406, 242.80137634277344]

tag_size = 0.164

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
        self.video_subscriber = self.create_subscription(Image, 'camera/color/image_raw', self.video_callback, 10)

        self.video_subscriber # prevent unused variable warning

        self.at_detector = Detector(searchpath=['apriltags'],
                              families='tag36h11',
                              nthreads=1,
                              quad_decimate=1.0,
                              quad_sigma=0.0,
                              refine_edges=1,
                              decode_sharpening=0.25,
                              debug=0)
       
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
    

    def video_callback(self, data):
        """
        Callback function.
        """    
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(data)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray_frame, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)
        frame_rgb = frame[:, :, ::-1].copy()    
        
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image message        
        # Display image
        if tags!=[]:
          v1 = (int(tags[0].corners[0][0]), int(tags[0].corners[0][1]))
          v2 = (int(tags[0].corners[1][0]), int(tags[0].corners[1][1]))
          v3 = (int(tags[0].corners[2][0]), int(tags[0].corners[2][1]))
          v4 = (int(tags[0].corners[3][0]), int(tags[0].corners[3][1]))
          pts = np.array([v1,v2,v3,v4], np.int32)
          pts = pts.reshape((-1,1,2))
          center_x = int(tags[0].center[0])
          center_y = int(tags[0].center[1])
          
          frame_rgb = cv2.polylines(frame_rgb,[pts],True,(0,0,255), 2)
          frame_rgb = cv2.circle(frame_rgb, (center_x, center_y), 3, (0, 0, 255), -1)
          """
          font = cv2.FONT_HERSHEY_SIMPLEX
          org = (center_x - 100, center_y - 50)
          fontScale = 0.9
          frame_rgb = cv2.putText(frame_rgb, "Mannaggia a Violante", org, font, fontScale, (0, 0, 255), 2, cv2.LINE_AA)
          """
          pose_R = tags[0].pose_R
          pose_t = tags[0].pose_t
          r = R.from_matrix(pose_R)
          self.get_logger().info(f"""
          Pose: 
          {pose_t.transpose()}
          Rot:
          {r.as_rotvec()}""")

        scale_percent = 200 # percent of original size
        width = int(frame_rgb.shape[1] * scale_percent / 100)
        height = int(frame_rgb.shape[0] * scale_percent / 100)
        dim = (width, height)
          
        # resize image
        frame_rgb = cv2.resize(frame_rgb, dim, interpolation = cv2.INTER_AREA)
        cv2.imshow("Video", frame_rgb)
        cv2.waitKey(1)    
        
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  node = VideoStreamerNode()
  node.get_logger().info("Video streamer node has started")

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