import rclpy.node

from jvr_robot.hardware.simulated.SG90Servo import SG90Servo
from jvr_robot.hardware.simulated.UltrasoneSensor import UltrasoneSensor
from jvr_robot.hardware.simulated.Motor import Motor

# ###########################################
# The simulated hardware

# Distance sweep servo and ultrasone sensor
def get_distance_sweep(n : rclpy.node.Node):
    clock = n.get_clock()
    sweep_servo = SG90Servo(clock)
    sweep_sensor = UltrasoneSensor(clock)
    return sweep_servo, sweep_sensor

# left and right motor
def get_motors(n : rclpy.node.Node):
    clock = n.get_clock()
    motor_left = Motor(clock, "left")
    motor_right = Motor(clock, "right")
    return motor_left, motor_right