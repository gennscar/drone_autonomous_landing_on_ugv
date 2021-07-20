#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt64


class NumAnchorsNode(Node):
    def __init__(self):
        super().__init__("numAnchorsNode")

        # region Parameters
        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RATE', None),
                ('QUEUE_SIZE', None),
                ('N', None)
            ]
        )

        # Parameters initialization
        self.RATE = self.get_parameter('RATE').value
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        self.N = self.get_parameter('N').value
        # endregion

        # Publishers initialization
        self.numAnchorsPub = self.create_publisher(UInt64, "N", self.QUEUE_SIZE)

        # Control loop timer
        self.timer = self.create_timer(1 / self.RATE, self.timerCallback)

    # region Callbacks
    def timerCallback(self):
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
