# *ros2-robotic-manipulation*
Welcome to the ROS2 Manipulator Arm Workflow repository! This repository showcases a comprehensive workflow for robotic manipulation using ROS2, with a focus on manipulator arm-related packages.

<p align="center">
  <img src="https://img.shields.io/badge/ROS 2-humble-blueviolet"/>
  <img src="https://img.shields.io/badge/PyTorch-2.0-red"/>
  <img src="https://img.shields.io/badge/TensorFlow-2.11-orange"/>
</p>

## Quick Start
- Clone the repo and build the docker image from Dockerfile present in repo
```bash
# clone the repo to desired directory
git clone git@github.com:kumar-sanjeeev/ros2-robotic-manipulation.git

# build the docker image 
docker build -t <desired_docker_image_name> . 

# launch the docker container using docker-run 
docker-run --name <conatiner_name> --mwd -it -e DOCKER_UID=$(id -u) -e DOCKER_GID=$(id -g) -e DOCKER_USER=$(id -un) --no-gpu --no-rm <desired_docker_image_name>:latest
```

## Info about ROS2 pkgs of mybot
- [ ] **ROS2 PKG** : [mybot_description](mybot_description):: This package provides a comprehensive description of the mybot robot.
