from rclpy.node import Node

import jvr_helpers.utils

from jvr_interfaces.msg import TalkMsg

import jvr_basic.ITalker


class Listener(Node):

    def __init__(self):
        node_name = jvr_helpers.utils.node_name(self)
        super().__init__(node_name)

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
    jvr_helpers.utils.run_node(node_type, args)


if __name__ == '__main__':
    main()
