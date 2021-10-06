#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PrintInformationNode(Node):
    def __init__(self):
        super().__init__("printInformationNode")

        # region Parameters
        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('QUEUE_SIZE', None)
            ]
        )

        # Parameters initialization
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        # endregion

        # Subscribers initialization
        self.vehiclesInfoSub = self.create_subscription(String, "/vehiclesInfo", self.vehiclesInfoCallback, self.QUEUE_SIZE)

    # region Callbacks
    def vehiclesInfoCallback(self, msg):
        self.get_logger().info(msg.data)
    # endregion


def main():
    rclpy.init(args=None)
    node = PrintInformationNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
