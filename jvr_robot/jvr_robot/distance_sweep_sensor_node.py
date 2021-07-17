import rclpy
import rclpy.timer
from rclpy.node import Node
import jvr_robot.utils
from jvr_robot.IObjectDetector import IObjectDetector
from jvr_interfaces.msg import ObjectDetection


# https://github.com/adafruit/Adafruit_Python_PCA9685/
# sudo pip install adafruit-pca9685
import Adafruit_PCA9685
# import RPi.GPIO as GPIO
import math
import time

SERVO_PIN = 15
SERVO_LEFT_PWM = 420
SERVO_RIGHT_PWM = 180
SERVO_LEFT_ANGLE = math.radians(-45)
SERVO_RIGHT_ANGLE = math.radians(45)
SERVO_ROTATION_SPEED_PER_SEC = 0.3 / math.radians(60)  # see datasheet of servo


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
        pwm = map(new_position, SERVO_LEFT_ANGLE, SERVO_RIGHT_ANGLE,
                  SERVO_LEFT_PWM, SERVO_RIGHT_PWM)
        pwm = math.trunc(pwm) # make float to integer
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

        
class distance_sweep_sensor_node(Node):

    def __init__(self):
        # node
        node_name = __class__.__qualname__.lower()
        super().__init__(node_name)
        object_detected_topic = jvr_robot.utils.topic_name(IObjectDetector.object_detected)
        self.pub = self.create_publisher(ObjectDetection, object_detected_topic, 10) # TODO find out what 10 means.

    

    def measure(self):
        msg = ObjectDetection()
        msg.sensor_id = 1
        msg.time_stamp = self.get_clock().now().to_msg()
        self.get_logger().info(str(msg))
        self.pub.publish(msg)


def main(args=None):
    node_type = distance_sweep_sensor_node
    
    print('Hi from ' + node_type.__qualname__)

    rclpy.init(args=args)

    node = node_type()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
