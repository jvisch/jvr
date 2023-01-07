# TODO: select on startup by rosarg (of zoiets)

from jvr_robot.hardware.hardware_real import *
# from jvr_robot.hardware.hardware_simulate import *

import rclpy.node
import rclpy.duration
import rclpy.constants

import threading


class Motors():
    def __init__(self, n: rclpy.node.Node) -> None:
        self.left, self.right = get_motors(n)
        self.clock = n.get_clock()
        self.c = threading.Condition()
        n.create_guard_condition

    def _move_motors(self, left_power: float, right_power: float):
        self.left.move(left_power)
        self.right.move(right_power)

    def _stop_motors(self):
        self.left.stop()
        self.right.stop()

    def move(self, left_power: float, right_power: float, duration: rclpy.duration.Duration) -> bool:
        # starts moving, return false if movement was aborted, otherwise true
        self._move_motors(left_power, right_power)
        # wait for timeout or external stop
        with self.c:
            wait_time = duration.nanoseconds / rclpy.constants.S_TO_NS
            # TODO: waiting is on python timer, should be RosClock
            if self.c.wait(wait_time):
                # Got a signal (called from `stop()`)
                return False
            else:
                # Duration passed, so stop the motors first
                self._stop_motors()
                return True

    def stop(self):
        self._stop_motors()
        with self.c:
            self.c.notify()
