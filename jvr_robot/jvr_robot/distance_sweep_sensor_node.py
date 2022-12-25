import sys
import threading

import rclpy
import rclpy.timer
import rclpy.node
from rclpy.executors import ExternalShutdownException

import sensor_msgs.msg 


import jvr_robot.utils
import jvr_robot.IObjectDetector
import jvr_interfaces.msg


# https://github.com/adafruit/Adafruit_Python_PCA9685/
# sudo pip install adafruit-pca9685
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import math
import time

SERVO_PIN = 15
SERVO_LEFT_PWM = 420
SERVO_RIGHT_PWM = 180
SERVO_LEFT_ANGLE = math.radians(45)
SERVO_RIGHT_ANGLE = math.radians(-45)
SERVO_ROTATION_SPEED_PER_SEC = 0.3 / math.radians(60)  # see datasheet of servo

ULTRASONE_GPIO_TRIGGER = 20
ULTRASONE_GPIO_ECHO = 21
ULTRASONE_SENSOR_FRAME_ID = 'ultrasone_sensor'

# Implementation based on tutorial https://osoyoo.com/2020/08/01/osoyoo-raspberry-pi-v2-0-car-lesson-3-obstacle-avoidance/


class SweepSensorServo:

    def __init__(self):
        # initialize servo
        self.pin = SERVO_PIN
        self.servo = Adafruit_PCA9685.PCA9685()
        self.servo.set_pwm_freq(60)
        # set servo to middle
        self.current_position = SERVO_LEFT_ANGLE  # most extreme position
        self.move_to_radians(0)
        time.sleep(1)  # just wait 1 sec.

    def move_to_radians(self, new_position):
        if new_position < min(SERVO_LEFT_ANGLE, SERVO_RIGHT_ANGLE):
            raise ValueError('new_position less then minimum angle')
        if new_position > max(SERVO_LEFT_ANGLE, SERVO_RIGHT_ANGLE):
            raise ValueError('new_position greater then maximum angle')
        # map function

        def map(value, in_min, in_max, out_min, out_max):
            return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        # move servo
        pwm = map(new_position, SERVO_LEFT_ANGLE, SERVO_RIGHT_ANGLE, SERVO_LEFT_PWM, SERVO_RIGHT_PWM)
        pwm = math.trunc(pwm)  # make float to integer

        self.servo.set_pwm(self.pin, 0, pwm)
        # calculate wait time (servo has no feedback capability)
        wait_time = abs(self.current_position - new_position) * SERVO_ROTATION_SPEED_PER_SEC
        wait_time = wait_time + .1  # add a small amount to be sure the servo is stopped
        time.sleep(wait_time)
        # calculate current position on pwm, because the real pwm was trunctated
        self.current_position = map(pwm, SERVO_LEFT_PWM, SERVO_RIGHT_PWM, SERVO_LEFT_ANGLE, SERVO_RIGHT_ANGLE)

    def move_to_degrees(self, new_position):
        pos = math.degrees(new_position)
        self.move_to_radian(pos)

    def get_current_position_radians(self):
        return self.current_position

    def get_current_position_degrees(self):
        return math.degrees(self.current_position)


class SweepSensorUltrasone:

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(ULTRASONE_GPIO_TRIGGER, GPIO.OUT)  # Trigger
        GPIO.setup(ULTRASONE_GPIO_ECHO, GPIO.IN)      # Echo

    def measure(self):
        # This function measures a distance
        GPIO.output(ULTRASONE_GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(ULTRASONE_GPIO_TRIGGER, False)
        start = time.time()
        while GPIO.input(ULTRASONE_GPIO_ECHO) == 0:
            start = time.time()
        while GPIO.input(ULTRASONE_GPIO_ECHO) == 1:
            stop = time.time()
        elapsed = stop-start
        # snelheid van het geluid +/- 343m/s
        distance = (elapsed / 2) * 343
        return distance


class distance_sweep_sensor_node(rclpy.node.Node):

    def __init__(self):
        # node
        node_name = __class__.__qualname__.lower()
        super().__init__(node_name)
        object_detected_topic = jvr_robot.utils.topic_name(jvr_robot.IObjectDetector.IObjectDetector.object_detected)
        # Measure distance
        self.servo = SweepSensorServo()
        self.sensor = SweepSensorUltrasone()
        self.pub = self.create_publisher(jvr_interfaces.msg.ObjectDetection, object_detected_topic, 10) # TODO find out what 10 means.
        self.measuring_thread = threading.Thread(target=self.measuring, daemon=True)
        self.measuring_thread.start()

    def measure(self):
        # Create new message
        msg = jvr_interfaces.msg.ObjectDetection()
        
        # Angle of servo
        msg.angle = self.servo.get_current_position_radians()
        
        # Object (Range)
        msg.object.radiation_type = sensor_msgs.msg.Range.ULTRASOUND
        msg.object.header.frame_id = ULTRASONE_SENSOR_FRAME_ID
        msg.object.header.stamp = self.get_clock().now().to_msg()
        # from specs of HC-SR04
        msg.object.min_range = 0.02 # 2cm
        msg.object.max_range = 5.00 # 500cm
        msg.object.field_of_view = math.radians(30)
        # measured values
        msg.object.range = self.sensor.measure()

        self.get_logger().info(str(msg))
        self.pub.publish(msg)

    def measuring(self):
        def constrain(value, min, max):
            if value < min:
                return min
            if value > max:
                return max
            return value
        left = min(SERVO_LEFT_ANGLE, SERVO_RIGHT_ANGLE)
        right = max(SERVO_LEFT_ANGLE, SERVO_RIGHT_ANGLE)
        while True:
            direction = left
            while direction <= right:
                self.servo.move_to_radians(direction)
                self.measure()
                direction = direction + math.radians(5)

            direction = right
            while direction >= left:
                self.servo.move_to_radians(direction)
                self.measure()
                direction = direction - math.radians(5)

def main(args=None):
    node_type = distance_sweep_sensor_node

    print('Hi from ' + node_type.__qualname__)

    rclpy.init(args=args)

    try:
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit()
    finally:
        rclpy.try_shutdown()
        node.destroy_node()



if __name__ == '__main__':
    main()

# let op,
# /dev/i2c/ is niet te benaderen voor ander gebruikers, doe:
#     1. sudo apt install i2c-tools
#     2. sudo groupadd i2c (group may exist already)
#     3. sudo chown :i2c /dev/i2c-1 (or i2c-0)
#     4. sudo chmod g+rw /dev/i2c-1
#     5. sudo usermod -aG i2c ubuntu
#     6. sudo reboot now
