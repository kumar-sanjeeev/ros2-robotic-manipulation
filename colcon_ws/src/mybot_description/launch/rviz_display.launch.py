from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # declare the argument for the launch file
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("mybot_description"), "urdf", "mybot.urdf.xacro"),
        description="Absolute path to the robot URDF File"
    )

    # convert the xacro.urdf to urdf format using command xacro
    # in order to set the launch file parameters while it is starting, we are using LaunchConfiguration
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    # start the robot state publisher node, to publish robot model over the topic
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description }],
        output="screen"
    )

    # start the joint state publisher gui node to move the robot joints
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    # start the rviz node to visulaize the robot
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("mybot_description"), "rviz", "rviz_config.rviz")]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_node, 
        rviz_node
    ])