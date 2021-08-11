# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from jvr_interfaces.msg import TalkMsg

NODE_NAME = 'listener'
TOPIC_NAME = 'talk'


class Listener(Node):

    def __init__(self, namespace = __package__):
        super().__init__(NODE_NAME, namespace=namespace)
        self.subscription = self.create_subscription(
            TalkMsg,
            TOPIC_NAME,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('[{0}]: msg "{1}"'.format(msg.id, msg.content))


def main(args=None):
    node_type = Listener
    print('Hi from ' + node_type.__qualname__)
    
    rclpy.init(args=args)

    node = node_type()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
