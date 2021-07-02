# Implementation of interfaces

## Emergency component

    IEmergency
        deactivate_engines() : void -->  /Emergency/IEmergency/deactivate_engines  std_msg::Empty
        activate_engines() : void   -->  /Emergency/IEmergency/activate_engines  std_msg::Empty
        panic() : void              -->  /Emergency/IEmergency/panic  std_msg::Empty

### IEmergency

-   deactivate_engines() : void

      ----------- ----------------------------------------------------------------------------------------------------------------------
      topic       /Emergency/IEmergency/deactivate_engines

      msg / srv   [common_interfaces::std_srvs::Empty.srv](https://github.com/ros2/common_interfaces/blob/foxy/std_srvs/srv/Empty.srv)

      type        service
      ----------- ----------------------------------------------------------------------------------------------------------------------

-   activate_engines() : void

      ----------- ----------------------------------------------------------------------------------------------------------------------
      topic       /Emergency/IEmergency/activactivate_engines

      msg / srv   [common_interfaces::std_srvs::Empty.srv](https://github.com/ros2/common_interfaces/blob/foxy/std_srvs/srv/Empty.srv)

      type        service
      ----------- ----------------------------------------------------------------------------------------------------------------------

-   panic() : void --> std_msg::Empty

      ----------- ----------------------------------------------------------------------------------------------------------------------
      topic       /Emergency/IEmergency/panic

      msg / srv   [common_interfaces::std_srvs::Empty.srv](https://github.com/ros2/common_interfaces/blob/foxy/std_srvs/srv/Empty.srv)

      type        service
      ----------- ----------------------------------------------------------------------------------------------------------------------
