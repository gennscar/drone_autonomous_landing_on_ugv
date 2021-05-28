
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from dt_apriltags import Detector
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleAttitude
from scipy.spatial.transform import Rotation as R

camera_params = [277.19135641132203, 277.19135641132203, 160.5, 120.5]
tag_size = 0.793 # act_tag/full tag

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
        self.video_subscriber = self.create_subscription(Image, '/camera/image_raw', self.video_callback, 10)
                
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

        self.drone_orientation = np.array([0,0,0,1])
        self.ground_truth = np.array([0,0,0])
        self.drone_orientation_subscriber = self.create_subscription(
            VehicleAttitude, "/VehicleAttitude_PubSubTopic", self.callback_drone_orientation, 10) 
        self.ground_truth_subscriber = self.create_subscription(
            Point, "ground_truth", self.callback_ground_truth, 10)       
        self.tag_pose_publisher = self.create_publisher(Point, "AprilTag_estimator/estimated_pos", 10)
        self.apriltag_error_publisher = self.create_publisher(Point, "AprilTag_estimator/error", 10)

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
          """font = cv2.FONT_HERSHEY_SIMPLEX
          org = (center_x - 100, center_y - 50)
          fontScale = 0.9
          frame_rgb = cv2.putText(frame_rgb, "THOT DETECTED", org, font, fontScale, (0, 0, 255), 2, cv2.LINE_AA)
          """
          pose_R = tags[0].pose_R
          pose_t = tags[0].pose_t
          pose_t = np.array([pose_t[0][0], pose_t[1][0], -pose_t[2][0]])
          #self.rot_camera2local = R.from_quat(self.drone_orientation)
          #pose_t = self.rot_camera2local.apply(pose_t)
          
          tag_pose = Point()
          tag_pose.x = pose_t[0]
          tag_pose.y = pose_t[1]
          tag_pose.z = pose_t[2]
          self.tag_pose_publisher.publish(tag_pose)

          err_ = Point()
          err_.x = pose_t[0] - self.ground_truth[0]
          err_.y = pose_t[1] - self.ground_truth[1]
          err_.z = pose_t[2] - self.ground_truth[2]
          print(f"""{pose_t[2]}, {self.ground_truth[2]}""")
          self.apriltag_error_publisher.publish(err_)

        scale_percent = 200 # percent of original size
        width = int(frame_rgb.shape[1] * scale_percent / 100)
        height = int(frame_rgb.shape[0] * scale_percent / 100)
        dim = (width, height)
          
        # resize image
        frame_rgb = cv2.resize(frame_rgb, dim, interpolation = cv2.INTER_AREA)

        cv2.imshow("Video", frame_rgb)
        cv2.waitKey(1)    

    def callback_drone_orientation(self, msg):
        self.drone_orientation = msg.q
    def callback_ground_truth(self, msg):
        self.ground_truth = np.array([msg.x, msg.y, msg.z])

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