# *ros2-robotic-manipulation*
Welcome to the ROS2 Manipulator Arm Workflow repository! This repository showcases a comprehensive workflow for robotic manipulation using ROS2, with a focus on manipulator arm-related packages.

<img src="https://img.shields.io/badge/ROS 2-humble-blueviolet"/>   ![Docker Badge](https://img.shields.io/badge/Docker-2496ED?style=flat-square&logo=docker&logoColor=white)

## Quick Start Steps
### Prerequiste 
- Install the [Docker](https://docs.docker.com/engine/install/ubuntu/) in your system.
- (Optional- if not using Docker Container) Install [ros2-humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) Distro in your system.
- Install [docker-run](https://github.com/ika-rwth-aachen/docker-run) cli tool to build the docker container from the docker image.

### Prerequiste Installation check
- **For Docker**

    Run the following command in terminal
    - ```bash 
       docker run hello-world
         ```
    - expected output :  if docker installation is proper
        ```
        Hello from Docker!
        This message shows that your installation appears to be working correctly.
        ```
- **For ROS2-Humble (optional: only if using system installed ROS2)** 
  - Run the following command in terminal

    The ros2 doctor command performs a series of checks to ensure your ROS 2 environment is properly configured. If all checks pass, it indicates that your ROS 2 Humble installation is in good shape.
    - ```bash 
        ros2 doctor 
        ```

    - expected output : 
            All 5 checks passed

### System setup : to run this repo code
- Clone the repo and build the docker image from Dockerfile present in repo
  ```bash
  # clone the repo to desired directory
  git clone git@github.com:kumar-sanjeeev/ros2-robotic-manipulation.git

  # build the docker image 
  docker build -t <desired_docker_image_name> . 

  # launch the docker container using docker-run 
  docker-run --name <conatiner_name> --mwd -it -e DOCKER_UID=$(id -u) -e DOCKER_GID=$(id -g) -e DOCKER_USER=$(id -un) --no-gpu --no-rm <desired_docker_image_name>:latest
  ```
  **Info :** to find what this **docker-run** command is doing visit [docker-run](https://github.com/ika-rwth-aachen/docker-run) and [docker-ros-ml-images](https://github.com/ika-rwth-aachen/docker-ros-ml-images) repositories.

- Now you have docker container (name: <container_name> ) running in your system. You can start and exit this container using following commands

    - To start the container :

        ```bash
         docker start <container_name/container_id>
         ```
    - To interact with the running container terminal :

        ```bash
        docker exec -it <container_name/container_id> /bin/bash 
        ```
- Build the colcon workspace
    - Container opens up at location where you will see following files
      ```bash
        colcon_ws  Dockerfile  README.md
      ```
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

## Info about ROS2 pkgs of mybot
- [x] **ROS2 PKG** : [mybot_description](colcon_ws/src/mybot_description/):: This package provides a comprehensive description of the custom mybot robot.

