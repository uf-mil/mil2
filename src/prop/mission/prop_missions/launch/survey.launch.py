"""
Survey the course, or a rectangle from the boat, with the grid planner, against
the running boat (or sim). Needs localization started, for /fromLL and map.

    ros2 launch prop_missions survey.launch.py
    ros2 service call /survey/start std_srvs/srv/Trigger
    ros2 service call /survey/stop std_srvs/srv/Trigger

Nothing moves until ~/start. Settings are in config/survey.yaml, and any can be
changed with ros2 param set before starting.
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
        "survey.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="prop_missions",
                executable="survey",
                parameters=[
                    config,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                    },
                ],
                output="screen",
            ),
        ],
    )
