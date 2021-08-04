#!/usr/bin/env python
from math import sin, cos, pi
import rclpy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from sensor_msgs.msg import JointState

import time

# This script positions the robot somewhere in the world
# based on WoR-world demo


# if __name__ == '__main__':
# 	rospy.init_node('robot_position_publisher')

# 	br = tf.TransformBroadcaster()
# 	rate = rospy.Rate(50)
# 	rotation = tf.transformations.quaternion_from_euler(0, 0, 0)
# 	position = (x,y,z)

# 	while not rospy.is_shutdown():
# 		br.sendTransform(position,rotation,rospy.Time.now(),"base_link","world")
# 		rate.sleep()


def main(args=None):
    print('Hi from ' + __file__)

    rclpy.init(args=args)
    n = rclpy.create_node('robot_position_publisher')


    pub =  n.create_publisher(JointState, 'joint_states', 10)
    br = tf2_ros.TransformBroadcaster(n)
    rotation = euler_to_quaternion(0, 0, 0)
    position = Vector3(x=0.0, y=0.0, z=0.0)

    rate = n.create_rate(10)

    while rclpy.ok():
        now = n.get_clock().now().to_msg()
        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'world'
        odom_trans.child_frame_id = 'chassis'
        odom_trans.header.stamp = now
        odom_trans.transform.rotation = rotation
        odom_trans.transform.translation = position

        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = ['joint_name']
        joint_state.position = [0.0]

        br.sendTransform(odom_trans)
        # pub.publish(joint_state)
        
        # rate.sleep()
        time.sleep(1)

# copied from ros2 tf2 tutorial
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


if __name__ == '__main__':
    main()
