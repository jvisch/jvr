import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from jvr_interfaces.msg import TalkMsg

NODE_NAME = 'talker'
TOPIC_NAME = 'talk'
TIMER_PERIOD = 0.5  # seconds


class Talker(Node):

    def __init__(self, namespace=__package__):
        super().__init__(NODE_NAME, namespace=namespace)
        self.publisher_ = self.create_publisher(TalkMsg, TOPIC_NAME, 10)

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.i += 1
        msg = TalkMsg()
        msg.id = self.i
        msg.content = 'publishing: "%s"' % msg.id
        self.publisher_.publish(msg)
        self.get_logger().info(msg.content)


def main(args=None):
    node_type = Talker
    print('Hi from ' + node_type.__qualname__)

    rclpy.init(args=args)

    try:
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit()
    finally:
        rclpy.try_shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
