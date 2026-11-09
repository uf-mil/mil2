"""
Task 1, Safe Passage, against the running boat (or sim) stack.

    ros2 launch prop_missions safe_passage.launch.py

Start it before the run: it waits for the OCS to pass on RunStart. Settings are
in config/safe_passage.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("prop_missions"),
        "config",
        "safe_passage.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odometry",
                default_value="/odometry/filtered/local",
                description="Odometry in the buoys' frame: the planner's.",
            ),
            DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="prop_missions",
                executable="safe_passage",
                parameters=[
                    config,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                    },
                ],
                remappings=[
                    ("odometry", LaunchConfiguration("odometry")),
                    ("gps/fix", LaunchConfiguration("gps_topic")),
                ],
                output="screen",
            ),
        ],
    )
