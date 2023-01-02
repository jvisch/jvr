import board
import busio
import adafruit_pca9685
import RPi

import jvr_robot.hardware.SG90Servo
import jvr_robot.hardware.UltrasoneSensor

# ###########################################
# Constants
PCA_PIN_SWEEP_SERVO = 15  # Servo Ultrasone sensor

GPIO_ULTRASONE_TRIGGER = 20  # Ultrasone pulse (start measure)
GPIO_ULTRASONE_ECHO = 21  # Ultrasone echo (measure complete)

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
sweep_servo = jvr_robot.hardware.SG90Servo.SG90Servo(
    pca.channels[PCA_PIN_SWEEP_SERVO]
)
sweep_sensor = jvr_robot.hardware.UltrasoneSensor.UltrasoneSensor(
    GPIO_ULTRASONE_TRIGGER,
    GPIO_ULTRASONE_ECHO
)

# left and right motor
## TODO