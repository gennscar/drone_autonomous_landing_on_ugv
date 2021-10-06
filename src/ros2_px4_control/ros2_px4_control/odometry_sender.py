#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from px4_msgs.msg import Timesync, VehicleVisualOdometry


SENDER_DT = 0.1
QUEUE_SIZE = 10


class OdometrySender(Node):
    """This node send the odometry information from another topic to PX4 visual
    odometry topic. It can be used to enhance the state estimation of the drone.
    """

    def __init__(self):
        super().__init__("OdometrySender")

        # Variables declaration
        self.timestamp_ = 0

        # Visual odometry message
        self.vis_ = VehicleVisualOdometry()
        self.vis_.q[0] = float("NaN")
        self.vis_.q_offset[0] = float("NaN")
        self.vis_.pose_covariance[15] = float("NaN")
        self.vis_.rollspeed = float("NaN")
        self.vis_.pitchspeed = float("NaN")
        self.vis_.yawspeed = float("NaN")
        self.vis_.velocity_covariance[15] = float("NaN")

        # Parameters declaration
        self.odometry_sub_name_ = self.declare_parameter("odometry_sub", None)

        # Retrieve parameter values and check validity
        self.odometry_sub_name_ = self.get_parameter(
            "odometry_sub").get_parameter_value().string_value

        if(self.odometry_sub_name_ == ""):
            self.get_logger().error("Missing odometry sub name")
            return

        # Subscribers initialization
        self.timesync_sub_ = self.create_subscription(
            Timesync, "Timesync_PubSubTopic",
            self.callback_timesync, QUEUE_SIZE
        )
        self.position_odometry_sub_ = self.create_subscription(
            PoseWithCovarianceStamped, self.odometry_sub_name_ + "/EstimatedPosition",
            self.callback_position_odometry, QUEUE_SIZE
        )
        self.velocity_odometry_sub_ = self.create_subscription(
            TwistWithCovarianceStamped, self.odometry_sub_name_ + "/EstimatedVelocity",
            self.callback_velocity_odometry, QUEUE_SIZE
        )

        # Publishers initialization
        self.visual_odometry_pub_ = self.create_publisher(
            VehicleVisualOdometry, "VehicleVisualOdometry_PubSubTopic",
            QUEUE_SIZE
        )

        self.get_logger().info("Node has started")

    def callback_timesync(self, msg):
        """This callback retrieve the timesync value from PX4.

        Args:
            msg (px4_msgs.msg.Timesync): Timesync message.
        """
        self.timestamp_ = msg.timestamp

    def callback_position_odometry(self, msg):
        """This callback retrieve the odometry information from a Pose sub and
        forward them to PX4 module through visual odometry topic

        Args:
            msg (geometry_msgs.msg.PoseWithCovarianceStamped): The pose with
            covariance and timestamp
        """

        self.vis_.timestamp = self.timestamp_
        self.vis_.timestamp_sample = self.timestamp_
        self.vis_.local_frame = VehicleVisualOdometry.LOCAL_FRAME_NED
        self.vis_.velocity_frame = VehicleVisualOdometry.LOCAL_FRAME_NED

        # Position
        self.vis_.x = msg.pose.pose.position.y
        self.vis_.y = msg.pose.pose.position.x
        self.vis_.z = -msg.pose.pose.position.z

        self.vis_.pose_covariance[0] = 0.0
        self.vis_.pose_covariance[6] = 0.0
        self.vis_.pose_covariance[11] = 0.0

        self.visual_odometry_pub_.publish(self.vis_)

    def callback_velocity_odometry(self, msg):
        """This callback retrieve the odometry information from a Pose sub and
        forward them to PX4 module through visual odometry topic

        Args:
            msg (geometry_msgs.msg.PoseWithCovarianceStamped): The pose with
            covariance and timestamp
        """

        self.vis_.timestamp = self.timestamp_
        self.vis_.timestamp_sample = self.timestamp_
        self.vis_.local_frame = VehicleVisualOdometry.LOCAL_FRAME_NED
        self.vis_.velocity_frame = VehicleVisualOdometry.LOCAL_FRAME_NED

        # Velocity
        self.vis_.vx = msg.twist.twist.linear.y
        self.vis_.vy = msg.twist.twist.linear.x
        self.vis_.vz = -msg.twist.twist.linear.z

        self.vis_.velocity_covariance[0] = 0.0
        self.vis_.velocity_covariance[6] = 0.0
        self.vis_.velocity_covariance[11] = 0.0

        self.visual_odometry_pub_.publish(self.vis_)


def main(args=None):
    rclpy.init(args=args)
    node = OdometrySender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
