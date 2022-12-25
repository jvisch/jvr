import sys
from math import sin, cos, pi

import rclpy
import rclpy.node
from rclpy.executors import ExternalShutdownException

import tf2_ros

import geometry_msgs.msg
import sensor_msgs.msg

import jvr_interfaces.msg
import jvr_robot.IObjectDetector
import jvr_robot.utils

# This script positions the robot somewhere in the world
# based on WoR-world demo.

TOPIC_OBJECTDETECTION_RANGE = 'jvr_objectdection_range'

class robot_position_publisher(rclpy.node.Node):

    def __init__(self):
        # node initialization
        node_name = __class__.__qualname__.lower()
        super().__init__(node_name)
        # init members
        self.br = tf2_ros.TransformBroadcaster(self)
        self.rotation = self.euler_to_quaternion(0, 0, 0)
        self.position = geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)

        topic_object_detected = jvr_robot.utils.topic_name(
            jvr_robot.IObjectDetector.IObjectDetector.object_detected)

        self.object_detected = self.create_subscription(
            jvr_interfaces.msg.ObjectDetection,
            topic_object_detected,
            self.object_detected_callback, 10)

        self.joint_pub = self.create_publisher(sensor_msgs.msg.JointState, 'joint_states', 10)
        self.range_pub = self.create_publisher(sensor_msgs.msg.Range, TOPIC_OBJECTDETECTION_RANGE, 10)

    def object_detected_callback(self, msg: jvr_interfaces.msg.ObjectDetection):
        # SweepSensor (servo and ultrasone as joint)
        # position of sensor (angle)
        joint_state = sensor_msgs.msg.JointState()
        joint_state.name = ["SweepSensorServo"]
        joint_state.header.stamp = msg.object.header.stamp # self.get_clock().now().to_msg()
        joint_state.position = [msg.angle]
        self.joint_pub.publish(joint_state)
        # Detected object (Range)
        self.range_pub.publish(msg.object)
        
        # publish robot position and poses
        odom_trans = geometry_msgs.msg.TransformStamped()
        odom_trans.header.frame_id = 'world'
        odom_trans.child_frame_id = 'chassis'
        odom_trans.header.stamp = msg.object.header.stamp # self.get_clock().now().to_msg()
        odom_trans.transform.rotation = self.rotation
        odom_trans.transform.translation = self.position
        self.br.sendTransform(odom_trans)

    def euler_to_quaternion(self, roll, pitch, yaw):  # copied from ros2 tf2 tutorial
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - \
            cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + \
            sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - \
            sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + \
            sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return geometry_msgs.msg.Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    node_type = robot_position_publisher

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
