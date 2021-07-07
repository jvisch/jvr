import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from jvr_robot.Emergency import Emergency, IEmergency


instance = Emergency()


class IEmergencyNode(Node):

    def __init__(self):
        # node
        node_name = instance.__class__.__qualname__.lower()
        super().__init__(node_name)

        # IEmergency
        #  panic
        panic_topic = "{node}/" + IEmergency.panic.__qualname__.replace('.', '/').lower()
        self.panic = self.create_service(Empty, panic_topic, self.panic_callback)
        # deactivate_motors
        deactivate_motors_topic = "{node}/" + IEmergency.deactivate_motors.__qualname__.replace('.', '/').lower()
        self.deactivate_motors = self.create_service(Empty, deactivate_motors_topic, self.deactivate_motors_callback)
        # def activate_motors
        activate_motors_topic = "{node}/" + IEmergency.activate_motors.__qualname__.replace('.', '/').lower()
        self.deactivate_motors = self.create_service(Empty, activate_motors_topic, self.activate_motors_callback)

    def panic_callback(self, request, response):
        self.get_logger().info("panic_callback")
        # instance.panic()
        return response

    def deactivate_motors_callback(self, request, response):
        self.get_logger().info("deactivate_motors_callback")
        # instance.deactivate_motors()
        return response
        
    def activate_motors_callback(self, request, response):
        self.get_logger().info("activate_motors_callback")
        # instance.activate_motors()
        return response



def main(args=None):
    print('Hi from jvr_robot.')
    rclpy.init(args=args)

    emergency = IEmergencyNode()

    rclpy.spin(emergency)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
