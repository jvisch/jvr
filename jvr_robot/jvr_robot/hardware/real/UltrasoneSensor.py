import RPi
import time

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
