# Components

## Emergency component

### Description

Responsible for emergency situations like collision, emergency breaks
etc. If an emergency occurs it will halt the robot and if necessary
shuts down the system.

### Messages

--none--

### Interfaces

#### IEmergency

    IEmergency
        deactivate_motors() : void
        activate_motors() : void
        panic() : void

-   `deactivate_motors()`

    Deactivates the motors so the robot will stop moving. The motors
    will be kept deactivated until `activate_motors()` is called.

-   `activate_motors()`

    Activates the motores again, so the robot is able to move again.

-   `panic()`

    Shutsdown the complete system, used in panic situations.

#### IObjectDetection

    IObjectDetection
        object_detected(ObjectDetectionMsg)

## DistanceSweepSensor

### Description

Distance sensor that constantly checks it surrounding for obstacles.
Keeps publishing its data `ObjectDetectionMsg` to the system. It also
provides functions for clients to ask current data.

When the component is initialized it starts detecting objects and starts
broadcasting them.

### Messages

#### ObjectDetectionMsg

Message containing information about a object detection.

    ObjectDetectionMsg
        SensorID   : uint16
        TimeStamp  : builtin_interfaces::Time
        Object     : sensor_msgs::Range

-   `SensorId`

    Unique ID of the sensor.

-   `TimeStamp`

    Timestamp of the measurement

-   `Object`

    Data about de object detection
    <https://github.com/ros2/common_interfaces/blob/foxy/sensor_msgs/msg/Range.msg>

### Interfaces

#### IObjectDetectionSweep

Get a range of detections --todo--
