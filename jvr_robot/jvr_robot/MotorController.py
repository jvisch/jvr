from jvr_robot.IMotorController import IMotorController


class MotorController(IMotorController):
    def __init__(self) -> None:
        super().__init__()

    ## IMotorController
    def deactivate_motors(self) -> None:
        raise NotImplementedError

    def activate_motors(self) -> None:
        raise NotImplementedError

    def panic(self) -> None:
        raise NotImplementedError

    ## IObjectDetector
    ## TODO object_detected(ObjectDetectionMsg)