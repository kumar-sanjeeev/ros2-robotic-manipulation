# Base Image from ika/rwth repo Ref: https://github.com/ika-rwth-aachen/docker-ros-ml-images/tree/main
FROM rwthika/ros2:humble-desktop-full

# Update the package list and install required ros2 humble pkgs
RUN apt-get update && apt-get install -y \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-gazebo-ros \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-moveit \
    ros-humble-urdf-tutorial