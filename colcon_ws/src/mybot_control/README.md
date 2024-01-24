# My Bot Description

This ROS2 package is used to control the robotic arm using the ros2 control. It configures the controllers
to be used to control the arms, via the `yaml` config file.

## Package Structure

The package Tree structure is as follows:

```bash
.
├── CMakeLists.txt
├── config
│   └── mybot_controller.yaml
├── launch
│   └── controller.launch.py
├── package.xml
└── README.md
└── urdf
```

## Quick Step to Launch the ROS2 control

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
- Launch the gazebo simulation

    ```bash
    ros2 launch mybot_description rviz_display.launch.py
    ```

- Launch the ros2 control

    ```bash
    ros2 launch mybot_control controller.launch.py 
    ```

- Output video

https://github.com/kumar-sanjeeev/ros2-robotic-manipulation/assets/62834697/260c2714-cad9-4c6a-89d1-051c513a4221
