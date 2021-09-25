
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Range
from ros2_px4_interfaces.msg import Yaw

import csv

# open the file in the write mode
KF_pos_estimator_0_ = open('KF_pos_estimator_0.csv', 'w')
LS_pos_estimator_ = open('LS_pos_estimator.csv', 'w')
LS_norot_pos_estimator_ = open('LS_norot_pos_estimator.csv', 'w')
range_sensor_ = open('range_sensor.csv', 'w')
yaw_sensor_ = open('yaw_sensor.csv', 'w')

class TopicRecorder(Node):

    def __init__(self):
        super().__init__("topic_recorder_node")

        self.starting_time_ = self.get_clock().now().to_msg()

        # subscribe to the topics
        self.KF_pos_estimator_0_subscriber_ = self.create_subscription(
            Odometry, "/KF_pos_estimator_0/estimated_pos", self.callback_KF_pos_estimator_0, 3)
        self.LS_pos_estimator_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, "/LS_uwb_estimator/estimated_pos", self.callback_LS_pos_estimator, 3)
        self.LS_norot_pos_estimator_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, "/LS_uwb_estimator/norot_pos", self.callback_LS_norot_pos_estimator, 3)
        self.range_sensor_subscriber_ = self.create_subscription(
            Range, "/drone/DistanceSensor_PubSubTopic", self.callback_range_sensor, 3)
        self.yaw_sensor_subscriber_ = self.create_subscription(
            Yaw, "/yaw_sensor/estimated_yaw", self.callback_yaw_sensor, 3)

        # create the csv writers
        self.KF_pos_estimator_0_writer_ = csv.writer(KF_pos_estimator_0_)
        self.LS_pos_estimator_writer_ = csv.writer(LS_pos_estimator_)
        self.LS_norot_pos_estimator_writer_ = csv.writer(LS_norot_pos_estimator_)
        self.range_sensor_writer_ = csv.writer(range_sensor_)
        self.yaw_sensor_writer_ = csv.writer(yaw_sensor_)

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

    def callback_range_sensor(self, msg):
        time_ = self.get_clock().now().to_msg()
        sec_ = (time_.sec - self.starting_time_.sec) + time_.nanosec*1e-9
        data_ = [sec_, msg.range]
        self.range_sensor_writer_.writerow(data_)

    def callback_yaw_sensor(self, msg):
        time_ = self.get_clock().now().to_msg()
        sec_ = (time_.sec - self.starting_time_.sec) + time_.nanosec*1e-9
        data_ = [sec_, msg.yaw]
        self.yaw_sensor_writer_.writerow(data_)

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
        range_sensor_.close()
        yaw_sensor_.close()

    finally:
        node.destroy_node()
        rclpy.shutdown() 
if __name__ == "main":
    main()
