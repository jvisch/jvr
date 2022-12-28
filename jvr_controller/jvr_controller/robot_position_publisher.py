from math import sin, cos

import rclpy
import rclpy.node

import tf2_ros

import geometry_msgs.msg
import sensor_msgs.msg

import jvr_helpers.utils
import jvr_interfaces.msg
import jvr_robot.IObjectDetector

import jvr_controller.IObjectDetection


# This script positions the robot somewhere in the world
# based on WoR-world demo.
class robot_position_publisher(rclpy.node.Node):

    def __init__(self):
        # node initialization
        node_name = jvr_helpers.utils.node_name(self)
        super().__init__(node_name)
        # init members
        self.br = tf2_ros.TransformBroadcaster(self)
        self.rotation = self.euler_to_quaternion(0, 0, 0)
        self.position = geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)

        # Subscribe to ObjectDetector
        topic_object_detected = jvr_helpers.utils.topic_name(
            jvr_robot.IObjectDetector.IObjectDetector.object_detected)

        self.object_detected = self.create_subscription(
            jvr_interfaces.msg.ObjectDetection,
            topic_object_detected,
            self.object_detected_callback, 10)

        # Publish position of joints
        TOPIC_JOINT_STATES = 'joint_states'
        self.joint_pub = self.create_publisher(
            sensor_msgs.msg.JointState, TOPIC_JOINT_STATES, 10)

        # Publish range of object (range ultrasone sensor)
        topic_object_range = jvr_helpers.utils.topic_name(
            jvr_controller.IObjectDetection.IObjectDetection.distance)
        self.range_pub = self.create_publisher(
            sensor_msgs.msg.Range, topic_object_range, 10)

    def object_detected_callback(self, msg: jvr_interfaces.msg.ObjectDetection):
        # SweepSensor (servo and ultrasone as joint)
        # position of sensor (angle)
        joint_state = sensor_msgs.msg.JointState()
        joint_state.name = ["SweepSensorServo"]
        joint_state.header.stamp = msg.object.header.stamp
        joint_state.position = [msg.angle]
        self.joint_pub.publish(joint_state)

        # Detected object (Range)
        self.range_pub.publish(msg.object)

        # publish robot position and poses
        odom_trans = geometry_msgs.msg.TransformStamped()
        odom_trans.header.frame_id = 'world'
        odom_trans.child_frame_id = 'chassis'
        odom_trans.header.stamp = msg.object.header.stamp
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
    jvr_helpers.utils.run_node(node_type, args)


if __name__ == '__main__':
    main()
