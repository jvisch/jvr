import rclpy
import rclpy.action
import rclpy.node
import rclpy.duration
import rclpy.action.server

import jvr_interfaces.action
import jvr_helpers.utils
import jvr_robot.IDrive
import jvr_robot.JvrRobotHardware


class drive_node(rclpy.node.Node):

    def __init__(self):
        # node
        node_name = jvr_helpers.utils.node_name(self)
        super().__init__(node_name)
        self.motors = jvr_robot.JvrRobotHardware.Motors(self)

        action_name = drive_topic = jvr_helpers.utils.topic_name(
            jvr_robot.IDrive.IDrive.move)
        self._action_server = rclpy.action.ActionServer(
            self,
            jvr_interfaces.action.Drive,
            action_name,
            self.execute_callback
        )

    def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        self.get_logger().info('Executing goal...')
        # extract values from message
        left = goal_handle.request.left
        right = goal_handle.request.right
        duration = rclpy.duration.Duration.from_msg(goal_handle.request.duration)
        # TODO: check the incoming values (0.3 <= p <= 1.0) and retrun 'not accepted'
        start_time = self.get_clock().now()
        self.motors.move(left, right, duration)

        # wait for the motors to stop
        end_time = self.get_clock().now()

        x = end_time - start_time

        
        # All succeeded
        self.get_logger().info("goal reached")
        goal_handle.succeed()
        result = jvr_interfaces.action.Drive.Result()
        result.total_duration = x.to_msg()
        return result


def main(args=None):
    node_type = drive_node
    jvr_helpers.utils.run_node(node_type, args)


if __name__ == '__main__':
    main()
