"""
cmd_wrench -> thrusters/left, thrusters/right, thrusters/bow.

    ros2 launch prop_controller thruster_mapper.launch.py

Sits between pid_controller and the thruster driver (prop_thrusters on the
boat, sim_thrusters.py under Gazebo). Thruster geometry and limits are in
config/controller.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("prop_controller"),
        "config",
        "controller.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="prop_controller",
                executable="thruster_mapper.py",
                name="thruster_mapper",
                parameters=[config],
                output="screen",
            ),
        ],
    )
