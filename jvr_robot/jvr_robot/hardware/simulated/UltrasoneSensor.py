import rclpy.clock
import rclpy.duration

# ###########################################
# Ultrasone sensor
class UltrasoneSensor:

    def __init__(self, clock : rclpy.clock.Clock):
        self.clock = clock
        self.distance = 0.2

    def measure(self):
        # wake up the physical sensor
        wait_ns = 0.00001 * rclpy.duration.S_TO_NS
        d = rclpy.duration.Duration(nanoseconds=wait_ns)
        self.clock.sleep_for(d)

        self.distance += .1
        if self.distance > 4:
            self.distance = 0.0
        # Measuring costs time
        # snelheid van het geluid +/- 343m/s
        wait_s = self.distance / 343
        wait_ns = wait_s * rclpy.duration.S_TO_NS
        d = rclpy.duration.Duration(nanoseconds=wait_ns)
        self.clock.sleep_for(d)
        
        return self.distance
