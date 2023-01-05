import time
import random

# ###########################################
# Ultrasone sensor
class UltrasoneSensor:

    def __init__(self):
        self.distance = 0.2

    def measure(self):
        self.distance += .1
        if self.distance > 4:
            self.distance = 0.0
        return self.distance
