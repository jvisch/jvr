# jvr

jv-robot

***TODO: explain what this repo is about*** - The robot - RPI -
computer/laptop (architecture)

## Prepare the robot

See [./docs/install.md](./docs/install.md)

## Install JVR

Create a ROS-workspace and get the packages.

1.  ROS 2 Humble [Creating a
    workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

    -   `mkdir -p ~/jvr_ws/src`
    -   `cd ~/jvr_ws/src`

2.  Clone this repo into `~/jvr_ws/src/`. The folderstructure will be
    something like this.

    ``` plain
    ~/jvr_ws
    └── src
        └── jvr
            ├── LICENSE
            ├── README.md
            ├── design
            │   └── .....
            ├── docs
            │   └── .....
            ├── jvr_basic
            │   └── .....
            ├── jvr_controller
            │   └── .....
            ├── jvr_interfaces
            │   └── ......
            └── jvr_robot
                └── .....
    ```

3.  Build the workspace

    1.  make sure ROS is sourced: `source /opt/ros/humble/setup.bash`
    2.  `cd ~/jvr_ws`
    3.  `colcon build`

4.  Check the build (local test)

    1.  Start 2 SSH terminals: `ssh <user-name>@<robot>`

    2.  Terminal 1:

        ``` plain
        cd ~/jvr_ws
        source ./install/setup.bash
        ros2 run jvr_basic talker
        ```

        Post messages on topic `/jvr_basic/talk`

    3.  Terminal 2:

        ``` plain
        cd ~/jvr_ws
        source ./install/setup.bash
        ros2 run jvr_basic listener
        ```

        Listens to message published on topic `/jvr_basic/talk`.
