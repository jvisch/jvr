import RPi
import math

import jvr_robot.hardware.real.Servo


class Motor(jvr_robot.hardware.real.Servo.Servo):
    MINIMAL_POWER = .3
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
            raise ValueError(f"Argument 'power' must be between -1.0 and +1.0 (value: '{power}')")
        if(value < Motor.MINIMAL_POWER):
            self.stop()
            raise ValueError(f"Argument 'power' must be at least +/- {Motor.MINIMAL_POWER} (value: '{power}')")
        # change percentage to pwm
        value = round(value * 0xFFF)
        # Start the engine
        self.duty_cycle(value)