import rclpy
import rclpy.node
from jvr_interfaces.msg import RobotInfo
import socket

class RobotInfoNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('robot_info', namespace=__package__)
        self.__publisher = self.create_publisher(RobotInfo, 'broadcast', 10)
        timer_period = 1  # seconds
        self.__timer = self.create_timer(timer_period, self.timer_callback)
        self.__counter = 0
        self.__host_name = socket.gethostname()

    def timer_callback(self):
        self.__counter += 1
        msg = RobotInfo()
        msg.id = self.__counter
        msg.host_name = self.__host_name
        self.__publisher.publish(msg)
        self.get_logger().info('{0}: id={1} hostname="{2}"'.format(self.__publisher.topic_name, msg.id, msg.host_name))

def main(args=None):
    rclpy.init(args=args)

    robot_info_node = RobotInfoNode() 
    rclpy.spin(robot_info_node)

if __name__ == '__main__':
    main()
