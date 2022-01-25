#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class NumAnchorsNode(Node):
    def __init__(self):
        """
        This method declares and initializes the parameters imported by "params.yaml". It declares useful variables, and
        a publisher. It creates a timer to regulate the frequency of the node.
        """

        super().__init__("numAnchorsNode")

        # region Parameters
        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RATE', None),
                ('QUEUE_SIZE', None),
                ('BEST_EFFORT', None),
                ('N', None)
            ]
        )

        # Parameters initialization
        self.RATE = self.get_parameter('RATE').value
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        self.BEST_EFFORT = self.get_parameter('BEST_EFFORT').value
        self.N = self.get_parameter('N').value
        # endregion

        # QOS initialization
        self.qosProfile = None
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

        # Publishers initialization
        self.numAnchorsPub = self.create_publisher(UInt64, "N", self.qosProfile)

        # Control loop timer
        self.timer = self.create_timer(1 / self.RATE, self.timerCallback)

    # region Callbacks
    def timerCallback(self):
        """
        This callback publishes, at constant rate, a message to inform every agent in the system about the quantity of
        drones in the formation. It keeps publishing until the ground station is functioning.
        """

        msg = UInt64()
        msg.data = self.N
        self.numAnchorsPub.publish(msg)
    # endregion


def main():
    rclpy.init(args=None)
    node = NumAnchorsNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
