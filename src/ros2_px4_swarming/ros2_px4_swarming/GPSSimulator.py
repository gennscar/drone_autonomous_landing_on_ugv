#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import Timesync, VehicleVisualOdometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class GPSSimulator(Node):
    def __init__(self):
        """
        This method declares and initializes the parameters imported by "params.yaml". It declares useful variables,
        publishers, and subscribers. It creates a timer to regulate the frequency of the node.
        """

        super().__init__("GPSSimulator")

        # region Parameters
        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('QUEUE_SIZE', None),
                ('BEST_EFFORT', None)
            ]
        )

        # Parameters initialization
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        self.BEST_EFFORT = self.get_parameter('BEST_EFFORT').value
        # endregion

        # region Variables declaration
        self.timestamp = 0
        self.qosProfile = None

        # QOS initialization
        if self.BEST_EFFORT:
            self.qosProfile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
            )
        else:
            self.qosProfile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
            )

        # Subscribers initialization
        self.timesyncSub = self.create_subscription(Timesync, "Timesync_PubSubTopic", self.timesyncCallback, self.qosProfile)

        # Publishers initialization
        self.visualOdometryPub = self.create_publisher(VehicleVisualOdometry, "VehicleVisualOdometry_PubSubTopic", self.qosProfile)

        # Timer initialization
        self.timer = self.create_timer(0.1, self.timerCallback)

    # region Callbacks
    def timesyncCallback(self, msg):
        """
        @param msg: message

        The timestamp is updated constantly.
        """

        self.timestamp = msg.timestamp

    def timerCallback(self):
        """
        It generates a message of type "VehicleVisualOdometry", that is published at constant rate on the relative
        topic. This is just a dummy message saying that the drone is located in (0, 0, 0), with velocity (0, 0, 0), but
        it allows the drone to be armed even if it does not receive information from its GNSS sensor. It is useful to
        test the arm and takeoff code portion in indoor situations (without mounting the propellers!).
        """

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
    # endregion


def main():
    rclpy.init(args=None)
    node = GPSSimulator()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
