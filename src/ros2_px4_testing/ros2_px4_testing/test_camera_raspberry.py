
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from dt_apriltags import Detector
import numpy as np
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
import time
import base64
import zmq
from px4_msgs.msg import VehicleLocalPosition

camera_params = [570.97921243, 573.57453263, 309.42078176, 239.79832231]
tag_size = 0.16 #16

context = zmq.Context()
footage_socket = context.socket(zmq.PUB)
footage_socket.connect('tcp://192.168.1.98:5555')

class VideoStreamerNode(Node):

    def __init__(self):

        super().__init__('node')

        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.video_timer = self.create_timer(0.1,self.acquire_video)

        self.at_detector = Detector(searchpath=['apriltags'],
                              families='tag36h11',
                              nthreads=1,
                              quad_decimate=1.0,
                              quad_sigma=0.0,
                              refine_edges=1,
                              decode_sharpening=0.25,
                              debug=0)

        self.estimator_topic_name_ = "/AprilTag_estimator/estimated_pos"
        self.tag_pose_publisher = self.create_publisher(Odometry, self.estimator_topic_name_,3)
        self.drone_heading_subscriber = self.create_subscription(VehicleLocalPosition, '/X500_3/VehicleLocalPosition_PubSubTopic', self.drone_heading_callback,3)

        self.rot_global2local = R.from_matrix([[0,1,0],[1,0,0],[0,0,-1]])
        self.drone_orientation = np.array([0,0,0,1])
        self.drone_heading = 0.0
        self.true_rot_local2chassis = R.from_quat([0,0,0,1])
        self.rot_90 = R.from_matrix([[0,-1,0],[1,0,0],[0,0,1]])
        self.rot_inv = R.from_matrix([[1,0,0],[0,1,0],[0,0,-1]])
        self.rot_m90 = R.from_matrix([[0,1,0],[-1,0,0],[0,0,1]])
        self.n_turns_ = 0
        self.old_yaw_NED_raw = 0.0

    def acquire_video(self):
      
      if (self.cap.isOpened() == False):
        print("Error")
        exit()
      else:
        ret,frame = self.cap.read()
        if ret == True:

          frame = self.detect_tag(frame)
          self.stream_video(frame)

        else:
            self.cap.release()
            cv2.destroyAllWindows()
          
    def detect_tag(self, frame):

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = self.at_detector.detect(gray_frame, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)
    
        if tags!=[]:

          v1 = (int(tags[0].corners[0][0]), int(tags[0].corners[0][1]))
          v2 = (int(tags[0].corners[1][0]), int(tags[0].corners[1][1]))
          v3 = (int(tags[0].corners[2][0]), int(tags[0].corners[2][1]))
          v4 = (int(tags[0].corners[3][0]), int(tags[0].corners[3][1]))
          pts = np.array([v1,v2,v3,v4], np.int32)
          pts = pts.reshape((-1,1,2))
          center_x = int(tags[0].center[0])
          center_y = int(tags[0].center[1])

          pose_R_B = tags[0].pose_R
          self.rot_camera2tag = R.from_matrix(pose_R_B)
          #self.rot_local2camera = R.from_quat(self.drone_orientation)
          self.rot_local2camera = R.from_euler('z',self.drone_heading,degrees = False)
          self.rot_global2tag = self.rot_local2camera*self.rot_90*(self.rot_camera2tag)    
          self.yaw_NED_raw =  (self.rot_global2tag.as_euler('xyz', degrees=True))[2]
          """
          if self.yaw_NED_raw >= -180.0 and self.yaw_NED_raw <= 180.0:
              if self.yaw_NED_raw - self.old_yaw_NED_raw > 300.0:
                  self.n_turns_ -= 1
              elif self.yaw_NED_raw - self.old_yaw_NED_raw < - 300.0:
                  self.n_turns_ += 1
              self.old_yaw_NED_raw = self.yaw_NED_raw

              self.yaw_NED = self.yaw_NED_raw + self.n_turns_*360.0 - 90
          """
          pose_t_B = np.array([- tags[0].pose_t[1][0], tags[0].pose_t[0][0], tags[0].pose_t[2][0]])
          self.pose_t_NED_rover = self.rot_local2camera.apply(np.array([pose_t_B[0], pose_t_B[1], pose_t_B[2]]))
          
          msg = Odometry()
          msg.header.frame_id = self.estimator_topic_name_
          msg.header.stamp = self.get_clock().now().to_msg()
          msg.pose.pose.position.x = self.pose_t_NED_rover[0]
          msg.pose.pose.position.y = self.pose_t_NED_rover[1]
          msg.pose.pose.position.z = self.pose_t_NED_rover[2]
          msg.pose.pose.orientation.w = self.yaw_NED_raw
          self.tag_pose_publisher.publish(msg)

          frame = cv2.polylines(frame,[pts],True,(0,0,255), 2)
          frame = cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)

          font = cv2.FONT_HERSHEY_SIMPLEX
          org = (center_x - 120, center_y + 80)
          fontScale = 0.5
          frame = cv2.putText(frame, f"x: {round(self.pose_t_NED_rover[0],1)}, y: {round(self.pose_t_NED_rover[1],1)}, z: {round(self.pose_t_NED_rover[2],1)}, w: {round(self.yaw_NED_raw,2)} ", org, font, fontScale, (0, 0, 255), 2, cv2.LINE_AA)
          
        return frame

    def drone_heading_callback(self, msg):

        self.drone_heading = msg.heading

    def stream_video(self, frame):

        encoded, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer)
        footage_socket.send(jpg_as_text)

    def take_photo(self,frame):

        cv2.imwrite('image'+time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())+'.jpg', frame)

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