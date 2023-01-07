import sys

import rclpy
import rclpy.executors


NODE_NAME_PREFIX = 'jvr'
TOPIC_NAME_PREFIX = 'jvr'


def topic_name(fn):
    name = fn.__qualname__.replace('.', '/').lower()
    return f'{TOPIC_NAME_PREFIX}/{name}'


def node_name(obj):
    name = obj.__class__.__qualname__.lower()
    return f'{NODE_NAME_PREFIX}_{name}'


def run_node(node_type, args, executor_type = None):
    print('Hi from ' + node_type.__qualname__)

    rclpy.init(args=args)

    node = node_type()
    executor = executor_type() if executor_type else None
    try:
        rclpy.spin(node,executor=executor)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit()
    finally:
        rclpy.try_shutdown()
        node.destroy_node()
