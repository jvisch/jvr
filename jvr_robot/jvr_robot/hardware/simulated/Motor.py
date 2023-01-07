

# #####################################33
# simulated motor
class Motor:
    def __init__(self, clock, id) -> None:
        self.clock = clock
        self.id = id

    def move(self, power: float):
        print(f"{self.id} motor: Broem broem '{power}'")

    def stop(self):
        print(f"{self.id} motor: Iiiiieeee stop")
        