import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from jvr_robot.MotorController import MotorController, IMotorController, IObjectDetector


instance = MotorController()


class motor_controller_node(Node):

    def __init__(self):
        # node
        node_name = 'motor_controller_node'
        component_name = instance.__class__.__qualname__.lower()
        super().__init__(node_name)

        # IMotorController
        #  panic
        panic_topic = component_name + '/' + IMotorController.panic.__qualname__.replace('.', '/').lower()
        self.panic = self.create_service(Empty, panic_topic, self.panic_callback)
        # deactivate_motors
        deactivate_motors_topic = component_name + '/' + IMotorController.deactivate_motors.__qualname__.replace('.', '/').lower()
        self.deactivate_motors = self.create_service(Empty, deactivate_motors_topic, self.deactivate_motors_callback)
        # def activate_motors
        activate_motors_topic = component_name + '/' + IMotorController.activate_motors.__qualname__.replace('.', '/').lower()
        self.deactivate_motors = self.create_service(Empty, activate_motors_topic, self.activate_motors_callback)

        # IObjectDetector
        # object_detected
        object_detected_topic = component_name + '/' + IObjectDetector.object_detected.__qualname__.replace('.', '/').lower()
        self.object_detected = self.create_service(Empty, object_detected_topic, self.object_detected_callback)

    def panic_callback(self, request, response):
        self.get_logger().info("panic_callback")
        instance.panic()
        return response

    def deactivate_motors_callback(self, request, response):
        self.get_logger().info("deactivate_motors_callback")
        instance.deactivate_motors()
        return response

    def activate_motors_callback(self, request, response):
        self.get_logger().info("activate_motors_callback")
        instance.activate_motors()
        return response

    def object_detected_callback(self, request, response):
        self.get_logger().info("object_detected_callback")
        instance.object_detected(request)
        return response


def main(args=None):
    t = motor_controller_node
    print('Hi from ' + t.__qualname__)
    rclpy.init(args=args)

    motor_controller = t()

    rclpy.spin(motor_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
