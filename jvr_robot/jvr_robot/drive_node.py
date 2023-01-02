import rclpy.node

import jvr_helpers.utils

import jvr_interfaces.msg 

import jvr_robot.IDrive

class Motor:
    pass


class drive_node(rclpy.node.Node):

    def __init__(self):
        # node
        node_name = jvr_helpers.utils.node_name(self)
        super().__init__(node_name)

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
        

def main(args=None):
    node_type = drive_node
    jvr_helpers.utils.run_node(node_type, args)


if __name__ == '__main__':
    main()