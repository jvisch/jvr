#!/usr/bin/env python
from math import sin, cos, pi
import rclpy
import rclpy.node
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3

import time

# This script positions the robot somewhere in the world
# based on WoR-world demo


class robot_position_publisher(rclpy.node.Node):

    def __init__(self):
        # node initialization
        node_name = __class__.__qualname__.lower()
        super().__init__(node_name)
        # init members
        self.br = tf2_ros.TransformBroadcaster(self)
        self.rotation = self.euler_to_quaternion(0, 0, 0)
        self.position = Vector3(x=0.0, y=0.0, z=0.0)
        # create callback
        self.timer = self.create_timer(1, self.position_callback)

    def position_callback(self):
        now = self.get_clock().now().to_msg()
        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'world'
        odom_trans.child_frame_id = 'chassis'
        odom_trans.header.stamp = now
        odom_trans.transform.rotation = self.rotation
        odom_trans.transform.translation = self.position

        self.br.sendTransform(odom_trans)

        time.sleep(1)

    def euler_to_quaternion(self, roll, pitch, yaw):  # copied from ros2 tf2 tutorial
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    node_type = robot_position_publisher

    print('Hi from ' + node_type.__qualname__)

    rclpy.init(args=args)

    node = node_type()
    # measuring_thread()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()