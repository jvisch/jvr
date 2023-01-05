import adafruit_pca9685


# ###########################################
# super class for servo's
class Servo:
    def __init__(self, channel: adafruit_pca9685.PWMChannel):
        self.channel = channel

    def duty_cycle(self, value: int):
        # value must be 12-bit
        self.channel.duty_cycle = (value << 4) + 0xf
