import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import jvr_helpers.utils

from jvr_interfaces.msg import TalkMsg

import jvr_basic.ITalker


class Listener(Node):

    def __init__(self):
        node_name = __class__.__qualname__.lower()
        super().__init__(node_name, namespace=__package__)

        talk_topic_name = jvr_helpers.utils.topic_name(jvr_basic.ITalker.ITalker.talk)
        self.subscription = self.create_subscription(
            TalkMsg,
            talk_topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
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
