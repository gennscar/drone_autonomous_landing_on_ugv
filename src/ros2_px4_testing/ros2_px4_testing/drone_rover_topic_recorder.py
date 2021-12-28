
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from ros2_px4_interfaces.msg import Yaw
from geometry_msgs.msg import Point
import csv

# open the file in the write mode
KF_pos_estimator_0_ = open('KF_pos_estimator_0.csv', 'w')
LS_pos_estimator_ = open('LS_pos_estimator.csv', 'w')
LS_norot_pos_estimator_ = open('LS_norot_pos_estimator.csv', 'w')
KF_pos_estimator_error_0_ = open('KF_pos_estimator_error_0.csv', 'w')
drone_local_position_ = open('drone_local_position.csv', 'w')

class TopicRecorder(Node):

    def __init__(self):
        super().__init__("topic_recorder_node")

        self.LS_rot_topic_ = self.declare_parameter("LS_rot_topic", "/LS_uwb_estimator/estimated_pos")
        self.LS_norot_topic_ = self.declare_parameter("LS_norot_topic", "/LS_uwb_estimator/norot_pos")
        self.KF_0_topic_ = self.declare_parameter("KF_0_topic", "/KF_pos_estimator_0/estimated_pos")
        self.KF_pos_estimator_error_0_topic_ = self.declare_parameter("KF_pos_estimator_error_0_topic", "/KF_pos_estimator_0/error")
        self.vehicle_namespace_ = self.declare_parameter("vehicle_namespace", "/drone")
        self.drone_local_position_topic_ = self.declare_parameter("drone_local_position_topic", "/VehicleLocalPosition_PubSubTopic")

        # Retrieve parameter values
        self.LS_rot_topic_ = self.get_parameter("LS_rot_topic").get_parameter_value().string_value
        self.LS_norot_topic_ = self.get_parameter("LS_norot_topic").get_parameter_value().string_value
        self.KF_0_topic_ = self.get_parameter("KF_0_topic").get_parameter_value().string_value
        self.KF_pos_estimator_error_0_topic_ = self.get_parameter("KF_pos_estimator_error_0_topic").get_parameter_value().string_value
        self.drone_local_position_topic_ = self.get_parameter("drone_local_position_topic").get_parameter_value().string_value
        self.vehicle_namespace_ = self.get_parameter("vehicle_namespace").get_parameter_value().string_value


        self.starting_time_ = self.get_clock().now().to_msg()

        # subscribe to the topics
        self.KF_pos_estimator_0_subscriber_ = self.create_subscription(
            Odometry, self.KF_0_topic_, self.callback_KF_pos_estimator_0, 3)
        self.LS_pos_estimator_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, self.LS_rot_topic_, self.callback_LS_pos_estimator, 3)
        self.LS_norot_pos_estimator_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, self.LS_norot_topic_, self.callback_LS_norot_pos_estimator, 3)
        self.KF_pos_estimator_error_0_subscriber_ = self.create_subscription(
            Point, self.KF_pos_estimator_error_0_topic_, self.callback_KF_pos_estimator_error_0, 3)
        self.drone_local_position_subscriber_ = self.create_subscription(
            VehicleLocalPosition, self.vehicle_namespace_ + self.drone_local_position_topic_, self.callback_drone_local_position, 3)

        # create the csv writers
        self.KF_pos_estimator_0_writer_ = csv.writer(KF_pos_estimator_0_)
        self.LS_pos_estimator_writer_ = csv.writer(LS_pos_estimator_)
        self.LS_norot_pos_estimator_writer_ = csv.writer(LS_norot_pos_estimator_)
        self.KF_pos_estimator_error_0_writer_ = csv.writer(KF_pos_estimator_error_0_)
        self.drone_local_position_writer_ = csv.writer(drone_local_position_)

        self.get_logger().info("Started recording topics")

    def callback_KF_pos_estimator_0(self, msg):
        time_ = self.get_clock().now().to_msg()
        sec_ = (time_.sec - self.starting_time_.sec) + time_.nanosec*1e-9
        data_ = [sec_, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                msg.pose.pose.orientation.w, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        self.KF_pos_estimator_0_writer_.writerow(data_)

    def callback_LS_pos_estimator(self, msg):
        time_ = self.get_clock().now().to_msg()
        sec_ = (time_.sec - self.starting_time_.sec) + time_.nanosec*1e-9
        data_ = [sec_, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.LS_pos_estimator_writer_.writerow(data_)

    def callback_LS_norot_pos_estimator(self, msg):
        time_ = self.get_clock().now().to_msg()
        sec_ = (time_.sec - self.starting_time_.sec) + time_.nanosec*1e-9
        data_ = [sec_, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.LS_norot_pos_estimator_writer_.writerow(data_)

    def callback_KF_pos_estimator_error_0(self, msg):
        time_ = self.get_clock().now().to_msg()
        sec_ = (time_.sec - self.starting_time_.sec) + time_.nanosec*1e-9
        data_ = [sec_, msg.x, msg.y, msg.z]
        self.KF_pos_estimator_error_0_writer_.writerow(data_)

    def callback_drone_local_position(self, msg):
        time_ = self.get_clock().now().to_msg()
        sec_ = (time_.sec - self.starting_time_.sec) + time_.nanosec*1e-9
        data_ = [sec_, msg.x, msg.y, msg.z]
        self.drone_local_position_writer_.writerow(data_)

def main(args=None):
    rclpy.init(args=args)
    node = TopicRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:

        # close csv files
        print('server stopped cleanly')
        KF_pos_estimator_0_.close()
        LS_pos_estimator_.close()
        LS_norot_pos_estimator_.close()
        KF_pos_estimator_error_0_.close()
        drone_local_position_.close()

    finally:
        node.destroy_node()
        rclpy.shutdown() 
if __name__ == "main":
    main()
