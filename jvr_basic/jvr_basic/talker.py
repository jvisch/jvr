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
from rclpy import publisher
from rclpy.node import Node

# from std_msgs.msg import String
from jvr_interfaces.msg import TalkMsg

NODE_NAME = 'talker'
TOPIC_NAME = 'talk'
TIMER_PERIOD = 0.5  # seconds

class Talker(Node):

    def __init__(self, namespace: str):
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
    rclpy.init(args=args)

    theTalker = Talker(__package__)

    rclpy.spin(theTalker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    theTalker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
