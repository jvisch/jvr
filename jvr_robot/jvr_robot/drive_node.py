import rclpy.node

import jvr_helpers.utils
import jvr_interfaces.msg 
import jvr_robot.IDrive
import jvr_robot.JvrRobotHardware

class drive_node(rclpy.node.Node):

    def __init__(self):
        # node
        node_name = jvr_helpers.utils.node_name(self)
        super().__init__(node_name)

        self.left = jvr_robot.JvrRobotHardware.motor_left
        self.right = jvr_robot.JvrRobotHardware.motor_right

        drive_topic = jvr_helpers.utils.topic_name(
            jvr_robot.IDrive.IDrive.move)
        self.subscription = self.create_subscription(
            jvr_interfaces.msg.Drive,
            drive_topic,
            self.drive_callback,
            10)
        self.subscription  # prevent unused variable warning

    def drive_callback(self, drive_msg):
        self.get_logger().info(str(drive_msg))
        power_left = drive_msg.left
        power_right = drive_msg.right
        self.left.move(power_left)
        self.right.move(power_right)
        
        napping_time = drive_msg.duration.sec + (drive_msg.duration.nanosec * 1e-9)
        import time
        time.sleep(napping_time)
        
        self.left.stop()
        self.right.stop()

        

def main(args=None):
    node_type = drive_node
    jvr_helpers.utils.run_node(node_type, args)


if __name__ == '__main__':
    main()