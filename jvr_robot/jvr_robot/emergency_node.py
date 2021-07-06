import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class EmergencyNode(Node):

    def __init__(self):
        super().__init__('emergency', namespace='NAMESPACEJV')
        self.srv = self.create_service(
            Empty,
            'knutsel',
            self.knutsel)

    def knutsel(self, request, response):
        self.get_logger().info('HALLOOOOOTJES')
        return response


def main(args=None):
    print('Hi from jvr_robot.')
    rclpy.init(args=args)

    emergency = EmergencyNode()

    rclpy.spin(emergency)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
