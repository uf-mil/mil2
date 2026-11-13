"""
The PID controller: goal/trajectory -> cmd_wrench.

    ros2 launch subjugator_controller_again pid_controller.launch.py
    ros2 launch subjugator_controller_again pid_controller.launch.py \
        param_file:=/path/to/gains.yaml odometry:=/sim/ground_truth

odometry is the estimate the controller closes the loop on, and the frame
goals arrive in. It comes up disabled: start-controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("subjugator_controller_again")
    default_param = os.path.join(pkg, "config", "pid_controller.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("param_file", default_value=default_param),
            DeclareLaunchArgument(
                "odometry",
                default_value="odometry/filtered/local",
                description="Odometry to control against.",
            ),
            Node(
                package="subjugator_controller_again",
                executable="subjugator_controller_again",
                name="pid_controller",
                parameters=[LaunchConfiguration("param_file")],
                remappings=[
                    ("odometry/filtered/local", LaunchConfiguration("odometry")),
                ],
                output="screen",
            ),
        ],
    )
