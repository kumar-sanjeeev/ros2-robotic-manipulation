from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # instantiate the argument for the launch file
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("mybot_description"), "urdf", "mybot.urdf.xacro"),
        description="Absolute path to the robot URDF File"
    )

    # set the env_var for Gazebo Simulation
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("mybot_description"), "share"))

    # convert the xacro.urdf to urdf format using command xacro
    # in order to set the launch file parameters while it is starting, we are using LaunchConfiguration
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    # start the robot state publisher node, to publish robot model over the topic
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description }]
    )

    # start gazebo server: by include the another launch file
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"))
    )

    # start gazebo client: vy inlcuding the another launch file
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"))
    )

    # spwan the robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "mybot", "-topic", "robot_description"]
    )

    return LaunchDescription([
        env_var,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])