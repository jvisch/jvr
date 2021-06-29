Components
==========

Emergency component
-------------------

Responsible for emergency situations like collision, emergency breaks
etc. If an emergency occurs it will halt the robot and if necessary
shuts down the system.

### Interfaces

    IEmergency
        deactivate_engines()
        activate_engines()
        shutdown()

-   `deactivate_engines()`

    Deactivates the motors so the robot will stop moving. The motors
    will be kept deactivated until `activate_engines()` is called.

-   `activate_engines()`

    Activates the motores again, so the robot is able to move again.

-   `shutdown()`

    Shutsdown the complete system, used in panic situations.

