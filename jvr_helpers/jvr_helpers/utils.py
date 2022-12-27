import sys

import rclpy
from rclpy.executors import ExternalShutdownException


def topic_name(function):
    return function.__qualname__.replace('.', '/').lower()


def node_name(obj):
    return obj.__class__.__qualname__.lower()


def run_node(node_type, args):
    print('Hi from ' + node_type.__qualname__)

    rclpy.init(args=args)

    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit()
    finally:
        rclpy.try_shutdown()
        node.destroy_node()
