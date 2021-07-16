from jvr_robot.IMotorController import IMotorController
from jvr_robot.IObjectDetector import IObjectDetector


class MotorController(IMotorController, IObjectDetector):
    def __init__(self) -> None:
        super().__init__()

    # IMotorController

    def deactivate_motors(self) -> None:
        raise NotImplementedError

    def activate_motors(self) -> None:
        raise NotImplementedError

    def panic(self) -> None:
        raise NotImplementedError

    # IObjectDetector

    def object_detected(self, ObjectDetectionMsg) -> None:
        raise NotImplementedError
