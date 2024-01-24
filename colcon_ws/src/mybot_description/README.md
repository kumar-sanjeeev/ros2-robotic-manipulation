# My Bot Description

This ROS2 package contains all the necessary files required to create the visual representation of
the custom 3 DOF robotic arm.

## Package Structure

The package Tree structure is as follows:

```bash
.
├── CMakeLists.txt
├── launch
├── meshes
├── package.xml
├── README.md
├── rviz
└── urdf
```

## Quick Step to Launch RViz Visualization

- Build the colcon workspace

    ```bash
    # source the ROS (system lvel)
    source /opt/ros/humble/setup.bash

    # cd into colcon_ws
    cd colcon_ws/

    # build the workspace
    colcon build

    # overlay the newly build packages
    source install/setup.bash
    ```
- Launch the Rviz visualization

    ```bash
    ros2 launch mybot_description rviz_display.launch.py
    ```

- Output video

https://github.com/kumar-sanjeeev/ros2-robotic-manipulation/assets/62834697/480a0131-e719-4309-9f4e-5c19743511da
