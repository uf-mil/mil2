"""
The thruster driver on its own.

    ros2 launch prop_thrusters thrusters.launch.py

Only useful on the boat: the driver talks to the Navigator board directly. In
simulation prop_gazebo stands in for it.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="prop_thrusters",
                executable="thrusters.py",
                name="thrusters",
                output="screen",
            ),
        ],
    )
