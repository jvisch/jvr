import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from jvr_robot.IMotorController import IMotorController
from jvr_robot.IObjectDetector import IObjectDetector
from jvr_interfaces.msg import ObjectDetection

class motor_controller_node(Node):

    def __init__(self):
        # node
        node_name = __class__.__qualname__.lower()
        super().__init__(node_name)

        # IMotorController
        #  panic
        panic_topic = IMotorController.panic.__qualname__.replace('.', '/').lower()
        self.panic = self.create_service(Empty, panic_topic, self.panic_callback)
        # deactivate_motors
        deactivate_motors_topic = IMotorController.deactivate_motors.__qualname__.replace('.', '/').lower()
        self.deactivate_motors = self.create_service(Empty, deactivate_motors_topic, self.deactivate_motors_callback)
        # def activate_motors
        activate_motors_topic = IMotorController.activate_motors.__qualname__.replace('.', '/').lower()
        self.deactivate_motors = self.create_service(Empty, activate_motors_topic, self.activate_motors_callback)

        # IObjectDetector
        # object_detected
        object_detected_topic = IObjectDetector.object_detected.__qualname__.replace('.', '/').lower()
        # self.object_detected = self.create_service(Empty, object_detected_topic, self.object_detected_callback)
        self.object_detected = self.create_subscription(
            ObjectDetection,
            object_detected_topic,
            self.object_detected_callback,
            10) # todo what does 10 mean?

    def panic_callback(self, request, response):
        self.get_logger().info("panic_callback")
        raise NotImplementedError
        return response

    def deactivate_motors_callback(self, request, response):
        self.get_logger().info("deactivate_motors_callback")
        raise NotImplementedError
        return response

    def activate_motors_callback(self, request, response):
        self.get_logger().info("activate_motors_callback")
        raise NotImplementedError
        return response

    def object_detected_callback(self, msg):
        self.get_logger().info("object_detected_callback")
        self.get_logger().debug(str(msg))


def main(args=None):
    node_type = motor_controller_node
    print('Hi from ' + node_type.__qualname__)
    
    rclpy.init(args=args)

    node = node_type()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
