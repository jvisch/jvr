import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from jvr_interfaces.msg import TalkMsg

import jvr_basic.ITalker

# todo (to the interface package)
def topic_name(function):
    return function.__qualname__.replace('.', '/').lower()

class Talker(Node):

    def __init__(self, timer_period=0.5):
        node_name = __class__.__qualname__.lower()
        super().__init__(node_name, namespace=__package__)
        
        talk_topic_name = topic_name(jvr_basic.ITalker.ITalker.talk)
        self.publisher_ = self.create_publisher(TalkMsg, talk_topic_name, 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
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
