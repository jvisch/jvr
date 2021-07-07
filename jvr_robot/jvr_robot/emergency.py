from jvr_robot.IEmergency import IEmergency


class Emergency(IEmergency):
    def __init__(self) -> None:
        super().__init__()

    ## IEmergency
    def deactivate_motors() -> None:
        raise NotImplemented

    def activate_motors() -> None:
        raise NotImplemented

    def panic() -> None:
        raise NotImplemented

    ## IObjectDetection
    ## TODO object_detected(ObjectDetectionMsg)