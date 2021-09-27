#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import Timesync, VehicleVisualOdometry


SENDER_DT = 0.1
QUEUE_SIZE = 10


class OdometrySender(Node):
    """This node send the odometry information from another topic to PX4 visual
    odometry topic. It can be used to enhance the state estimation of the drone.
    """

    def __init__(self):
        super().__init__("odometry_sender")

        # Variables declaration
        self.timestamp_ = 0

        # Parameters declaration
        self.odometry_sub_name_ = self.declare_parameter("odometry_sub", None)

        # Retrieve parameter values and check validity
        self.odometry_sub_name_ = self.get_parameter(
            "odometry_sub").get_parameter_value().string_value

        if(self.odometry_sub_name_ == ""):
            self.get_logger().error("Missing odometry sub name")
            return
        
        # Subscribers initialization        
        self.odometry_sub_ = self.create_subscription(
            PoseWithCovarianceStamped, self.odometry_sub_name_,
            self.callback_odometry, QUEUE_SIZE
        )

        # Publishers initialization
        self.visual_odometry_pub_ = self.create_publisher(
            VehicleVisualOdometry, "VehicleVisualOdometry_PubSubTopic", 
            QUEUE_SIZE
        )

        # Timer initialization
        self.timer_ = self.create_timer(SENDER_DT, self.callback_timer)

    def callback_timesync(self, msg):
        """This callback retrieve the timesync value from PX4.

        Args:
            msg (px4_msgs.msg.Timesync): Timesync message.
        """
        self.timestamp_ = msg.timestamp

    def callback_odometry(self, msg):
        """This callback retrieve the odometry information from a Pose sub and
        forward them to PX4 module through visual odometry topic

        Args:
            msg (geometry_msgs.msg.PoseWithCovarianceStamped): The pose with
            covariance and timestamp
        """

        msg = VehicleVisualOdometry()
        msg.timestamp = self.timestamp_
        msg.timestamp_sample = self.timestamp_
        msg.local_frame = VehicleVisualOdometry.LOCAL_FRAME_FRD
        msg.velocity_frame = VehicleVisualOdometry.LOCAL_FRAME_FRD

        # Position
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0

        msg.pose_covariance[0] = 1.0e-6
        msg.pose_covariance[1] = 0.0
        msg.pose_covariance[2] = 0.0
        msg.pose_covariance[6] = 1.0e-6
        msg.pose_covariance[7] = 0.0
        msg.pose_covariance[11] = 1.0e-6

        # Velocity
        msg.vx = float("NaN")
        msg.vy = float("NaN")
        msg.vz = float("NaN")

        msg.velocity_covariance[0] = 1.0e-6
        msg.velocity_covariance[1] = 0.0
        msg.velocity_covariance[2] = 0.0
        msg.velocity_covariance[6] = 1.0e-6
        msg.velocity_covariance[7] = 0.0
        msg.velocity_covariance[11] = 1.0e-6

        # Attitude
        msg.q[0] = float("NaN")
        msg.q_offset[0] = float("NaN")
        msg.pose_covariance[15] = float("NaN")

        # Rate
        msg.rollspeed = float("NaN")
        msg.pitchspeed = float("NaN")
        msg.yawspeed = float("NaN")
        msg.velocity_covariance[15] = float("NaN")

        self.visual_odometry_pub_.publish(msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = OdometrySender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
