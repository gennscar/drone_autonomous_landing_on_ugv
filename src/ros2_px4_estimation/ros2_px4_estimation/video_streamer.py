
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from dt_apriltags import Detector
from geometry_msgs.msg import Point, PointStamped
from px4_msgs.msg import VehicleAttitude
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R

k1 = 1.0
k2 = 1.0
camera_params = [277.19135641132203*k1, 277.19135641132203*k1, 160.5, 120.5]
tag_size = 0.793*k2 # act_tag/full tag


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
        self.video_subscriber = self.create_subscription(Image, '/camera/image_raw', self.video_callback, 1)
                
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
        self.uwb_position_wrt_chassis = [0.,0.,0.]

        self.drone_orientation_subscriber = self.create_subscription(VehicleAttitude, "/VehicleAttitude_PubSubTopic", self.callback_drone_orientation, 1) 
        #self.drone_orientation_subscriber = self.create_subscription(Odometry, "/uwb_sensor_iris/odom", self.callback_drone_orientation, 1)       
        self.tag_pose_publisher = self.create_publisher(Point, "AprilTag_estimator/estimated_pos", 1)
        self.uwb_position_wrt_chassis_subscriber = self.create_subscription(
            PointStamped, "/GN_10iter_uwb_estimator/estimated_pos", self.callback_uwb_position_wrt_chassis, 10)
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
          font = cv2.FONT_HERSHEY_SIMPLEX
          org = (center_x - 100, center_y - 50)
          fontScale = 0.9
          frame_rgb = cv2.putText(frame_rgb, "MATCH", org, font, fontScale, (0, 0, 255), 2, cv2.LINE_AA)
          
          pose_R = tags[0].pose_R
          pose_t = tags[0].pose_t

          self.rot_camera2chassis = R.from_matrix(pose_R)
          self.rot_local2camera = R.from_quat(self.drone_orientation)
          self.rot_90 = R.from_matrix([[0,-1,0],[1,0,0],[0,0,1]])
          self.rot_m90 = R.from_matrix([[0,1,0],[-1,0,0],[0,0,1]])
          self.rot_local2chassis = self.rot_m90*self.rot_local2camera*self.rot_90*(self.rot_camera2chassis.inv())
          self.uwb_position_wrt_chassis = - np.array(self.rot_local2chassis.apply(self.uwb_position_wrt_chassis))
          tag_pose = Point()
          tag_pose.x = self.uwb_position_wrt_chassis[0]
          tag_pose.y = self.uwb_position_wrt_chassis[1]
          tag_pose.z = self.uwb_position_wrt_chassis[2]
          self.tag_pose_publisher.publish(tag_pose)         

        scale_percent = 200 # percent of original size
        width = int(frame_rgb.shape[1] * scale_percent / 100)
        height = int(frame_rgb.shape[0] * scale_percent / 100)
        dim = (width, height)
          
        # resize image
        frame_rgb = cv2.resize(frame_rgb, dim, interpolation = cv2.INTER_AREA)

        cv2.imshow("Video", frame_rgb)
        cv2.waitKey(1)    

    def callback_uwb_position_wrt_chassis(self, msg):
        self.uwb_position_wrt_chassis = [msg.point.x, msg.point.y, msg.point.z]

    def callback_drone_orientation(self, msg):
        self.drone_orientation = np.array([msg.q[3], msg.q[0], msg.q[1], msg.q[2]])
        #self.drone_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

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