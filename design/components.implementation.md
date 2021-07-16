# Implementation of interfaces

## MotorController component

### IMotorController

-   deactivate_engines() : void

      -------- ----------------------------------------------------------------------------------------------------------------------
      topic:   /IMotorController/deactivate_engines

      msg /    [common_interfaces::std_srvs::Empty.srv](https://github.com/ros2/common_interfaces/blob/foxy/std_srvs/srv/Empty.srv)
      srv:     

      type:    service
      -------- ----------------------------------------------------------------------------------------------------------------------

-   activate_engines() : void

      -------- ----------------------------------------------------------------------------------------------------------------------
      topic:   /IMotorController/activactivate_engines

      msg /    [common_interfaces::std_srvs::Empty.srv](https://github.com/ros2/common_interfaces/blob/foxy/std_srvs/srv/Empty.srv)
      srv:     

      type:    service
      -------- ----------------------------------------------------------------------------------------------------------------------

-   panic() : void --> std_msg::Empty

      -------- ----------------------------------------------------------------------------------------------------------------------
      topic:   /IMotorController/panic

      msg /    [common_interfaces::std_srvs::Empty.srv](https://github.com/ros2/common_interfaces/blob/foxy/std_srvs/srv/Empty.srv)
      srv:     

      type:    service
      -------- ----------------------------------------------------------------------------------------------------------------------

## IObjectDetector

-   object_detected(ObjectDetectionMsg) : void

        ----------------- ------------------------------------------------
        topic:            /IObjectDetector/object_detected

        msg / srv:        jvr_interfaces::ObjectDetection.msg

        type:             listener
        ----------------- ------------------------------------------------
