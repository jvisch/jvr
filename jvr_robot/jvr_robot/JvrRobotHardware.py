# TODO: select on startup by rosarg (of zoiets)

# from jvr_robot.hardware.hardware_real import *
from jvr_robot.hardware.hardware_simulate import *

import rclpy.node
import rclpy.duration


class Motors():
    def __init__(self, n: rclpy.node.Node) -> None:
        self.left , self.right = get_motors(n)
        self.clock = n.get_clock()

    def move(self, left_power: float, right_power: float, duration: rclpy.duration.Duration):
        self.left.move(left_power)
        self.right.move(right_power)
        self.clock.sleep_for(duration)
