import rclpy
import rclpy.action
import rclpy.node

import jvr_interfaces.action
import jvr_helpers.utils
import jvr_robot.IDrive
import jvr_robot.JvrRobotHardware

class DriveActionServer(rclpy.node.Node):

    def __init__(self):
        # node
        node_name = jvr_helpers.utils.node_name(self)
        super().__init__(node_name)
        self.left_motor, self.right_motor = jvr_robot.JvrRobotHardware.get_motors(self)

        action_name = drive_topic = jvr_helpers.utils.topic_name(jvr_robot.IDrive.IDrive.move)
        self._action_server = rclpy.action.ActionServer(
            self,
            jvr_interfaces.action.Drive,
            action_name,
            self.execute_callback
        )

    def execute_callback(self, goal_handle:jvr_interfaces.action.Drive.Goal):
        self.get_logger().info('Executing goal...')
        
        # TODO: check the incoming values (0.3 <= p <= 1.0)

        # self.left_motor.move(goal_handle.left)
        # self.left_
        

        # All succeeded
        self.get_logger().info("goal reached")
        goal_handle.succeed()
        result = jvr_interfaces.action.Drive.Result()
        result.total_duration.sec = 42
        return result


def main(args=None):
    node_type = DriveActionServer
    jvr_helpers.utils.run_node(node_type, args)


if __name__ == '__main__':
    main()