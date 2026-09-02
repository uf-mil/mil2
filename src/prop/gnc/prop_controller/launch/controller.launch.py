"""
The autonomy stack: mission -> guidance -> thruster manager.

    ros2 launch prop_controller controller.launch.py mission:=square

Needs odometry/filtered/global from prop_localization and something listening on
thrusters/left and thrusters/right - the driver on the boat, or the Gazebo
bridge in simulation. The thruster manager sends the heartbeat itself, so do not
run a separate one alongside it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    config = pkg_share("prop_controller", "config", "controller.yaml")
    mission_file = PathJoinSubstitution(
        [
            pkg_share("prop_controller", "config", "missions"),
            [LaunchConfiguration("mission"), ".yaml"],
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission",
                default_value="square",
                description="Mission file in prop_controller/config/missions.",
            ),
            Node(
                package="prop_controller",
                executable="mission",
                name="mission",
                parameters=[mission_file],
                output="screen",
            ),
            Node(
                package="prop_controller",
                executable="guidance",
                name="guidance",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="prop_controller",
                executable="thruster_manager",
                name="thruster_manager",
                parameters=[config],
                output="screen",
            ),
        ],
    )
