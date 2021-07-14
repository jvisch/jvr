import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from jvr_robot.DistanceSweepSensor import DistanceSweepSensor

instance = DistanceSweepSensor()

class DistanceSweepSensorNode(Node):
    # node
    node_name = instance.__class__.__qualname__.lower()
    super().__init__(node_name)


def main(args=None):
    print('Hi from DistanceSweepSensorNode.')
    rclpy.init(args=args)

    node = DistanceSweepSensorNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()