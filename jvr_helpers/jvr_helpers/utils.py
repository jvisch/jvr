import sys

import rclpy
import rclpy.node
import rclpy.executors


NODE_NAME_PREFIX = 'jvr'
TOPIC_NAME_PREFIX = 'jvr'


def topic_name(fn):
    name = fn.__qualname__.replace('.', '/').lower()
    return f'{TOPIC_NAME_PREFIX}/{name}'


def node_name(obj, hidden_node=False):
    name = obj.__class__.__qualname__.lower()
    name = f'{NODE_NAME_PREFIX}_{name}'
    if hidden_node:
        name = rclpy.node.HIDDEN_NODE_PREFIX + name
    return name


def run_node(node_type, args, executor_type = None):
    print(f"Starting instance of '{node_type.__qualname__}'.")
    
    rclpy.init(args=args)
    node = node_type()
    print(f"Hi from  '{node.get_fully_qualified_name()}'.")
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
