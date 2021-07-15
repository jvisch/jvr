import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from jvr_robot.DistanceSweepSensor import DistanceSweepSensor


instance = DistanceSweepSensor()


class distance_sweep_sensor_node(Node):
    # node
    node_name = instance.__class__.__qualname__.lower()
    super().__init__(node_name)


def main(args=None):
    node_type = distance_sweep_sensor_node
    print('Hi from ' + node_type.__qualname__)

    rclpy.init(args=args)

    node = node_type()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
