# jvr_basic

Provides some basic functionality to test the ROS network.

## Interfaces

### ITalk

    ITalk
        Talk(TalkMsg) : void

-   `Talk(TalkMsg) : void`

    Receives a `TalkMsg` (package `jvr_interfaces`) so the ROS network
    can be tested.

## Components

### ListenerNode

-   Provided interfaces

    -   `ITalk`
        -   `Talk(TalkMsg)` -\> subscriber on `/italk/talk`

-   Required interfaces

    *none*

### TalkerNode

-   Provided interfaces

    *none*

-   Required interfaces

    -   `ITalk`
        -   `Talk(TalkMsg)` -\> publisher on `/italk/talk`
