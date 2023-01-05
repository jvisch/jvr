from jvr_robot.hardware_simulated.SG90Servo import SG90Servo
from jvr_robot.hardware_simulated.UltrasoneSensor import UltrasoneSensor
from jvr_robot.hardware_simulated.Motor import Motor

# ###########################################
# The simulated hardware

# Distance sweep servo and ultrasone sensor
sweep_servo = SG90Servo()
sweep_sensor = UltrasoneSensor()

# left and right motor
motor_left = Motor()
motor_right = Motor()