import board
import busio
import adafruit_pca9685
import RPi

import math
import time

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
# super class for servo's


class Channel:
    def __init__(self, channel: adafruit_pca9685.PWMChannel):
        self.channel = channel

    def duty_cycle(self, value: int):
        # value must be 12-bit
        self.channel.duty_cycle = (value << 4) + 0xf


# ###########################################
# Servo for ultrasone sensor
class SweepSensorServo(Channel):
    LEFT_PWM = 420
    RIGHT_PWM = 180
    LEFT_ANGLE = math.radians(45)
    RIGHT_ANGLE = math.radians(-45)
    ROTATION_SPEED_PER_SEC = 0.3 / math.radians(60)  # see datasheet of servo

    def __init__(self, index: int):
        c = pca.channels[index]
        super().__init__(c)
        # set servo to middle
        self.current_position = SweepSensorServo.LEFT_ANGLE  # most extreme position
        self.move_to_radians(0)
        time.sleep(1)  # just wait 1 sec.

    def move_to_radians(self, new_position):
        if new_position < min(SweepSensorServo.LEFT_ANGLE, SweepSensorServo.RIGHT_ANGLE):
            raise ValueError('new_position less then minimum angle')
        if new_position > max(SweepSensorServo.LEFT_ANGLE, SweepSensorServo.RIGHT_ANGLE):
            raise ValueError('new_position greater then maximum angle')

        # map function
        def map(value, in_min, in_max, out_min, out_max):
            return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        # move servo
        pwm = map(new_position, SweepSensorServo.LEFT_ANGLE, SweepSensorServo.RIGHT_ANGLE,
                  SweepSensorServo.LEFT_PWM, SweepSensorServo.RIGHT_PWM)
        pwm = math.trunc(pwm)  # make float to integer
        self.duty_cycle(pwm)

        # calculate wait time (servo has no feedback capability)
        wait_time = abs(self.current_position - new_position) * \
            SweepSensorServo.ROTATION_SPEED_PER_SEC
        wait_time = wait_time + .1  # add a small amount to be sure the servo is stopped
        time.sleep(wait_time)

        # calculate current position on pwm, because the real pwm was trunctated
        self.current_position = map(pwm, SweepSensorServo.LEFT_PWM, SweepSensorServo.RIGHT_PWM,
                                    SweepSensorServo.LEFT_ANGLE, SweepSensorServo.RIGHT_ANGLE)

    def move_to_degrees(self, new_position):
        pos = math.degrees(new_position)
        self.move_to_radian(pos)

    def get_current_position_radians(self):
        return self.current_position

    def get_current_position_degrees(self):
        return math.degrees(self.current_position)


# ###########################################
# Ultrasone sensor
class UltrasoneSensor:

    def __init__(self, trigger, echo):
        self.trigger = trigger
        self.echo = echo
        RPi.GPIO.setup(self.trigger, RPi.GPIO.OUT)  # Trigger
        RPi.GPIO.setup(self.echo, RPi.GPIO.IN)      # Echo

    def measure(self):
        # This function measures a distance
        RPi.GPIO.output(self.trigger, True)
        time.sleep(0.00001)
        RPi.GPIO.output(self.trigger, False)
        start = time.time()
        while RPi.GPIO.input(self.echo) == 0:
            start = time.time()
        while RPi.GPIO.input(self.echo) == 1:
            stop = time.time()
        elapsed = stop-start
        # snelheid van het geluid +/- 343m/s
        distance = (elapsed / 2) * 343
        return distance
