# Implementation of interfaces

## Emergency component

### IEmergency

-   deactivate_engines() : void

      -------- ----------------------------------------------------------------------------------------------------------------------
      topic:   /Emergency/IEmergency/deactivate_engines

      msg /    [common_interfaces::std_srvs::Empty.srv](https://github.com/ros2/common_interfaces/blob/foxy/std_srvs/srv/Empty.srv)
      srv:     

      type:    service
      -------- ----------------------------------------------------------------------------------------------------------------------

-   activate_engines() : void

      -------- ----------------------------------------------------------------------------------------------------------------------
      topic:   /Emergency/IEmergency/activactivate_engines

      msg /    [common_interfaces::std_srvs::Empty.srv](https://github.com/ros2/common_interfaces/blob/foxy/std_srvs/srv/Empty.srv)
      srv:     

      type:    service
      -------- ----------------------------------------------------------------------------------------------------------------------

-   panic() : void --> std_msg::Empty

      -------- ----------------------------------------------------------------------------------------------------------------------
      topic:   /Emergency/IEmergency/panic

      msg /    [common_interfaces::std_srvs::Empty.srv](https://github.com/ros2/common_interfaces/blob/foxy/std_srvs/srv/Empty.srv)
      srv:     

      type:    service
      -------- ----------------------------------------------------------------------------------------------------------------------

## IObjectDetection

- object_detected(ObjectDetectionMsg) : void

      -------- ----------------------------------------------------------------------------------------------------------------------
      topic:   /Emergency/IObjectDetection/object_detected

      msg /    jvr_interfaces::ObjectDetectionMsg.msg
      srv:     

      type:    listener
      -------- ----------------------------------------------------------------------------------------------------------------------

