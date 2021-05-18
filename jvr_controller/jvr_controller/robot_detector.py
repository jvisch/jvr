import rclpy
import rclpy.node
from jvr_interfaces.msg import RobotInfo

class RobotDetector(rclpy.node.Node):

    def __init__(self):
        super().__init__('robot_detector', namespace=__package__)
        self.__subscription = self.create_subscription(
            RobotInfo,
            '/jvr_robot/broadcast',
            self.broadcast_callback,
            10
        )

    def broadcast_callback(self, msg):
        print('Gevonden {0}: {1}'.format(msg.id, msg.host_name))


def main(args=None):
    rclpy.init(args=args)

    detector = RobotDetector()

    rclpy.spin(detector)
    
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
