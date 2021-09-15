#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import Timesync, VehicleVisualOdometry


class GPSSimulator(Node):
    def __init__(self):
        super().__init__("GPS_simulator")

        # Parameters declaration
        self.declare_parameters('queue_size', 10)

        # Parameters initialization
        self.queue_size_ = self.get_parameter(
            'queue_size').get_parameter_value().integer_value

        # Variables declaration
        self.timestamp = 0

        # Subscribers initialization
        self.timesyncSub = self.create_subscription(
            Timesync, "Timesync_PubSubTopic", self.timesyncCallback, self.queue_size_)

        # Publishers initialization
        self.visualOdometryPub = self.create_publisher(
            VehicleVisualOdometry, "VehicleVisualOdometry_PubSubTopic", self.queue_size_)

        # Timer initialization
        self.timer = self.create_timer(0.1, self.timerCallback)

    def timesyncCallback(self, msg):
        self.timestamp = msg.timestamp

    def timerCallback(self):
        msg = VehicleVisualOdometry()
        msg.timestamp = self.timestamp
        msg.timestamp_sample = self.timestamp
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
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vz = 0.0

        msg.velocity_covariance[0] = 1.0e-6
        msg.velocity_covariance[1] = 0.0
        msg.velocity_covariance[2] = 0.0
        msg.velocity_covariance[6] = 1.0e-6
        msg.velocity_covariance[7] = 0.0
        msg.velocity_covariance[11] = 1.0e-6

        # Attitude
        msg.q[0] = float('NaN')
        msg.q_offset[0] = float('NaN')
        msg.pose_covariance[15] = float('NaN')

        # Rate
        msg.rollspeed = float('NaN')
        msg.pitchspeed = float('NaN')
        msg.yawspeed = float('NaN')
        msg.velocity_covariance[15] = float('NaN')

        self.visualOdometryPub.publish(msg)


def main():
    rclpy.init(args=None)
    node = GPSSimulator()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
