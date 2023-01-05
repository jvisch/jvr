import jvr_robot.hardware_simulated.SG90Servo
import jvr_robot.hardware_simulated.UltrasoneSensor

# ###########################################
# The simulated hardware

# Distance sweep servo and ultrasone sensor
sweep_servo = jvr_robot.hardware_simulated.SG90Servo.SG90Servo()
sweep_sensor = jvr_robot.hardware_simulated.UltrasoneSensor.UltrasoneSensor()

# left and right motor
motor_left = None
motor_right = None