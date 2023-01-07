import threading

import rclpy
import rclpy.action
import rclpy.node
import rclpy.duration
import rclpy.action.server
import rclpy.executors

import jvr_interfaces.action
import jvr_helpers.utils
import jvr_robot.IDrive
import jvr_robot.JvrRobotHardware


class drive_node(rclpy.node.Node):

    def __init__(self):
        # node
        node_name = jvr_helpers.utils.node_name(self)
        super().__init__(node_name)
        self._motors = jvr_robot.JvrRobotHardware.Motors(self)

        action_name = drive_topic = jvr_helpers.utils.topic_name(
            jvr_robot.IDrive.IDrive.move)

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = rclpy.action.ActionServer(
            self,
            jvr_interfaces.action.Drive,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request: jvr_interfaces.action.Drive.Goal):
        self.get_logger().info('goal_callback')
        return rclpy.action.GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        self.get_logger().info('handle_accepted_callback')
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        # self.get_logger().info('Received cancel request')
        self.get_logger().info('cancel_callback')
        return rclpy.action.CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        # self.get_logger().info('Executing goal...')
        self.get_logger().info('execute_callback')
        # extract values from message
        left = goal_handle.request.left
        right = goal_handle.request.right
        duration = rclpy.duration.Duration.from_msg(goal_handle.request.duration)
        # TODO: check the incoming values (0.3 <= p <= 1.0) and retrun 'not accepted'
        start_time = self.get_clock().now()
        self._motors.move(left, right, duration)

        # wait for the motors to stop
        end_time = self.get_clock().now()
        total_duration = end_time - start_time

        if not goal_handle.is_active:
            self.get_logger().info('Goal aborted')
            return jvr_interfaces.action.Drive.Result()
        
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return jvr_interfaces.action.Drive.Result()

        # All succeeded
        self.get_logger().info("goal reached")
        goal_handle.succeed()
        result = jvr_interfaces.action.Drive.Result()
        result.total_duration = total_duration.to_msg()
        return result


def main(args=None):
    executor_type = rclpy.executors.MultiThreadedExecutor
    node_type = drive_node
    jvr_helpers.utils.run_node(node_type, args, executor_type=executor_type)


if __name__ == '__main__':
    main()
