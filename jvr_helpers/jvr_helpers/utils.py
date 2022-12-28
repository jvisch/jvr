import sys

import rclpy
from rclpy.executors import ExternalShutdownException


NODE_NAME_PREFIX = 'jvr'
TOPIC_NAME_PREFIX = 'jvr'

def topic_name(fn):
    name = fn.__qualname__.replace('.', '/').lower()
    return f'{TOPIC_NAME_PREFIX}/{name}'


def node_name(obj):
    name = obj.__class__.__qualname__.lower()
    return f'{NODE_NAME_PREFIX}_{name}'


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
