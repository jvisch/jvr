from rclpy.node import Node

import jvr_helpers.utils

from jvr_interfaces.msg import TalkMsg

import jvr_basic.ITalker


class Talker(Node):

    def __init__(self, timer_period=0.5):
        node_name = jvr_helpers.utils.node_name(self)
        super().__init__(node_name, namespace=__package__)

        talk_topic_name = jvr_helpers.utils.topic_name(
            jvr_basic.ITalker.ITalker.talk)
        self.publisher_ = self.create_publisher(TalkMsg, talk_topic_name, 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.i += 1
        msg = TalkMsg()
        msg.id = self.i
        msg.content = f'publishing: "{msg.id}"'
        self.publisher_.publish(msg)
        self.get_logger().info(msg.content)


def main(args=None):
    node_type = Talker
    jvr_helpers.utils.run_node(node_type, args)


if __name__ == '__main__':
    main()
