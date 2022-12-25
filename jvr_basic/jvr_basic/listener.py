import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from jvr_interfaces.msg import TalkMsg

NODE_NAME = 'listener'
TOPIC_NAME = 'talk'


class Listener(Node):

    def __init__(self, namespace=__package__):
        super().__init__(NODE_NAME, namespace=namespace)
        self.subscription = self.create_subscription(
            TalkMsg,
            TOPIC_NAME,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('[{0}]: msg "{1}"'.format(msg.id, msg.content))
        self.get_logger().info(f'{msg.id}: "{msg.content}"')


def main(args=None):
    node_type = Listener
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
