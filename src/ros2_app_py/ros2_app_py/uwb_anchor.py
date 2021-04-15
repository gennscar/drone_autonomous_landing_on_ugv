import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose


class UWBAnchor(Node):
    def __init__(self):
        super().__init__('uwb_anchor')

        self.publisher_ = self.create_publisher(Pose, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose()
        msg.position.x = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position.x)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = UWBAnchor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
