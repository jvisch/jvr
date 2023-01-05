import math
import time

# ###########################################
# Simulated Servo for ultrasone sensor


class SG90Servo():
    LEFT_PWM = 420
    RIGHT_PWM = 180
    LEFT_ANGLE = math.radians(45)
    RIGHT_ANGLE = math.radians(-45)
    ROTATION_SPEED_PER_SEC = 0.3 / math.radians(60)  # see datasheet of servo

    def __init__(self):
        # set servo to middle
        self.current_position = SG90Servo.LEFT_ANGLE  # most extreme position
        self.move_to_radians(0)
        time.sleep(1)  # just wait 1 sec.

    def move_to_radians(self, new_position):
        if new_position < min(SG90Servo.LEFT_ANGLE, SG90Servo.RIGHT_ANGLE):
            raise ValueError('new_position less then minimum angle')
        if new_position > max(SG90Servo.LEFT_ANGLE, SG90Servo.RIGHT_ANGLE):
            raise ValueError('new_position greater then maximum angle')

        # map function
        def map(value, in_min, in_max, out_min, out_max):
            return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        # move servo
        pwm = map(new_position, SG90Servo.LEFT_ANGLE, SG90Servo.RIGHT_ANGLE,
                  SG90Servo.LEFT_PWM, SG90Servo.RIGHT_PWM)
        pwm = math.trunc(pwm)  # make float to integer

        # calculate wait time (servo has no feedback capability)
        wait_time = abs(self.current_position - new_position) * \
            SG90Servo.ROTATION_SPEED_PER_SEC
        wait_time = wait_time + .1  # add a small amount to be sure the servo is stopped
        time.sleep(wait_time)

        # calculate current position on pwm, because the real pwm was trunctated
        self.current_position = map(pwm, SG90Servo.LEFT_PWM, SG90Servo.RIGHT_PWM,
                                    SG90Servo.LEFT_ANGLE, SG90Servo.RIGHT_ANGLE)

    def move_to_degrees(self, new_position):
        pos = math.degrees(new_position)
        self.move_to_radian(pos)

    def get_current_position_radians(self):
        return self.current_position

    def get_current_position_degrees(self):
        return math.degrees(self.current_position)
