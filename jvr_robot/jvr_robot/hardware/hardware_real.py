import board
import busio
import adafruit_pca9685
import RPi

import rclpy.node

from jvr_robot.hardware.real.SG90Servo import SG90Servo
from jvr_robot.hardware.real.UltrasoneSensor import UltrasoneSensor
from jvr_robot.hardware.real.Motor import Motor

# ###########################################
# Constants
PCA_PIN_SWEEP_SERVO = 15  # Servo Ultrasone sensor
PCA_PIN_MOTOR_LEFT = 0
PCA_PIN_MOTOR_RIGHT = 1

GPIO_ULTRASONE_TRIGGER = 20  # Ultrasone pulse (start measure)
GPIO_ULTRASONE_ECHO = 21  # Ultrasone echo (measure complete)

GPIO_LEFT_IN1 = 23  # left motor direction pin
GPIO_LEFT_IN2 = 24  # left motor direction pin
GPIO_RIGHT_IN1 = 27  # right motor direction pin
GPIO_RIGHT_IN2 = 22  # right motor direction pin


# ###########################################
# initialize all the hardware
i2c = busio.I2C(board.SCL, board.SDA)

pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 60

RPi.GPIO.setmode(RPi.GPIO.BCM)
RPi.GPIO.setwarnings(False)


# ###########################################
# The hardware

# Distance sweep servo and ultrasone sensor
def get_distance_sweep(n : rclpy.node.Node):
    clock = n.get_clock()
    # Distance sweep servo and ultrasone sensor
    servo = SG90Servo(
        pca.channels[PCA_PIN_SWEEP_SERVO]
    )
    sensor = UltrasoneSensor(
        GPIO_ULTRASONE_TRIGGER,
        GPIO_ULTRASONE_ECHO
    )

    return servo, sensor

# left and right motor
def get_motors(n : rclpy.node.Node):
    left = Motor(
        pca.channels[PCA_PIN_MOTOR_LEFT],
        GPIO_LEFT_IN1,
        GPIO_LEFT_IN2
    )
    right = Motor(
        pca.channels[PCA_PIN_MOTOR_RIGHT],
        GPIO_RIGHT_IN1,
        GPIO_RIGHT_IN2
    )
    return left, right
