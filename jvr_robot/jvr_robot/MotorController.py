from jvr_robot.IMotorController import IMotorController


class MotorController(IMotorController):
    def __init__(self) -> None:
        super().__init__()

    ## IMotorController
    def deactivate_motors() -> None:
        raise NotImplemented

    def activate_motors() -> None:
        raise NotImplemented

    def panic() -> None:
        raise NotImplemented

    ## IObjectDetector
    ## TODO object_detected(ObjectDetectionMsg)