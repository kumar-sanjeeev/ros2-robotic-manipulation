# My Bot Description

This ROS2 package provides the angle conversion utility in form of ros2 service which can be called by any node.
## Package Structure

The package Tree structure is as follows:

```bash
.
├── CMakeLists.txt
├── mybot_utils
│   ├── angle_conversion.py
│   └── __init__.py
├── package.xml
├── README.md
└── src
    └── angle_conversion.cpp
```

## How to use the angle conversion service

- Build the colcon workspace

    ```bash
    # source the ROS (system level)
    source /opt/ros/humble/setup.bash

    # cd into colcon_ws
    cd colcon_ws/

    # build the workspace
    colcon build

    # overlay the newly build packages
    source install/setup.bash
    ```
- Run the angle conversion service node.

    ```bash
    ros2 run mybot_utils angle_conversion
    ```

- Call the service

    ```bash
    rros2 service call /euler_to_quaternion mybot_msgs/srv/EulerToQuaternion "roll: 0.0
    pitch: 0.0
    yaw: 1.5" 
    ```

- Output video

https://github.com/kumar-sanjeeev/ros2-robotic-manipulation/assets/62834697/5b584d7f-d086-48bc-91d0-d73321ece3ce
