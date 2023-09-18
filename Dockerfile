# Base Image from athackst/dockerfiles repo Ref: https://github.com/athackst/dockerfiles/blob/main/ros2/humble.Dockerfile
FROM althack/ros2:humble-gazebo

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

