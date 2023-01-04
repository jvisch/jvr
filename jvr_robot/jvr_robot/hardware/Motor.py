import RPi
import math

import jvr_robot.hardware.Servo


class Motor(jvr_robot.hardware.Servo.Servo):

    def __init__(self, channel, IN1, IN2):
        super().__init__(channel)
        self.IN1 = IN1
        self.IN2 = IN2
        RPi.GPIO.setup(IN1, RPi.GPIO.OUT)
        RPi.GPIO.setup(IN2, RPi.GPIO.OUT)

    def write_gpio(self, value1, value2):
        RPi.GPIO.output(self.IN1, value1)
        RPi.GPIO.output(self.IN2, value2)

    def stop(self):
        self.write_gpio(RPi.GPIO.LOW, RPi.GPIO.LOW)
        self.duty_cycle(0)

    def forward(self):
        self.write_gpio(RPi.GPIO.LOW, RPi.GPIO.HIGH)

    def backward(self):
        self.write_gpio(RPi.GPIO.HIGH, RPi.GPIO.LOW)

    def move(self, power: float):
        # set direction
        if power >= 0:
            self.forward()
        else:
            self.backward()
        # Set power
        value = abs(power)
        #  power cannot exceed 100%
        if(value > 1.0):
            self.stop()
            raise ValueError(f"Argument 'power' must be between -1.0 and +1.0 (value: '{power}'")
        # less than 20%, the motors don't turn
        if value < 0.2:
            value = 0.2
        value = round(value * 0xFFF)
        # Start the engine
        self.duty_cycle(value)