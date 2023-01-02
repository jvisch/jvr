import sys
import threading

import rclpy
import rclpy.timer
import rclpy.node
from rclpy.executors import ExternalShutdownException

import sensor_msgs.msg

import jvr_helpers.utils
import jvr_interfaces.msg
import jvr_robot.IObjectDetector

import adafruit_pca9685
import board
import busio
import RPi

import math
import time

import jvr_robot.JvrRobotHardware

ULTRASONE_GPIO_TRIGGER = 20
ULTRASONE_GPIO_ECHO = 21
ULTRASONE_SENSOR_FRAME_ID = 'ultrasone_sensor'



class SweepSensorUltrasone:

    def __init__(self):
        RPi.GPIO.setmode(RPi.GPIO.BCM)
        RPi.GPIO.setwarnings(False)
        RPi.GPIO.setup(ULTRASONE_GPIO_TRIGGER, RPi.GPIO.OUT)  # Trigger
        RPi.GPIO.setup(ULTRASONE_GPIO_ECHO, RPi.GPIO.IN)      # Echo

    def measure(self):
        # This function measures a distance
        RPi.GPIO.output(ULTRASONE_GPIO_TRIGGER, True)
        time.sleep(0.00001)
        RPi.GPIO.output(ULTRASONE_GPIO_TRIGGER, False)
        start = time.time()
        while RPi.GPIO.input(ULTRASONE_GPIO_ECHO) == 0:
            start = time.time()
        while RPi.GPIO.input(ULTRASONE_GPIO_ECHO) == 1:
            stop = time.time()
        elapsed = stop-start
        # snelheid van het geluid +/- 343m/s
        distance = (elapsed / 2) * 343
        return distance


class distance_sweep_sensor_node(rclpy.node.Node):

    def __init__(self):
        # node
        node_name = jvr_helpers.utils.node_name(self)
        super().__init__(node_name)

        object_detected_topic = jvr_helpers.utils.topic_name(
            jvr_robot.IObjectDetector.IObjectDetector.object_detected)
        
        # Measure distance
        self.servo = jvr_robot.JvrRobotHardware.SweepSensorServo(jvr_robot.JvrRobotHardware.PIN_SWEEP_SERVO)
        self.sensor = SweepSensorUltrasone()
        # TODO find out what 10 means.
        self.pub = self.create_publisher(
            jvr_interfaces.msg.ObjectDetection, object_detected_topic, 10)
        self.measuring_thread = threading.Thread(
            target=self.measuring, daemon=True)
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
        msg.object.min_range = 0.02  # 2cm
        msg.object.max_range = 5.00  # 500cm
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
        
        left_angle = jvr_robot.JvrRobotHardware.SweepSensorServo.LEFT_ANGLE
        right_angle = jvr_robot.JvrRobotHardware.SweepSensorServo.RIGHT_ANGLE
        left = min( left_angle, right_angle)
        right = max(left_angle, right_angle)
        
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
    jvr_helpers.utils.run_node(node_type, args)


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
