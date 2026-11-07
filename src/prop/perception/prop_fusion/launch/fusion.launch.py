"""
Label the lidar's tracks with what the cameras see, and keep those labels on the
evidence grid so they outlast the tracks.

    ros2 launch prop_fusion fusion.launch.py
    ros2 launch prop_fusion fusion.launch.py debug_image:=true

Needs pcd's tracker, prop_vision's cameras.launch.py, the sensor frames from
sensors.launch.py and the grid planner's occupancy_grid. Labelled tracks come
out on classified_markers, and what the grid remembers on labelled_objects.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from prop_fusion.mounts import default_sensors_file


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("prop_fusion"),
        "config",
        "fusion.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensors_file",
                default_value=default_sensors_file(),
                description="The cameras to fuse are the ones named here.",
            ),
            DeclareLaunchArgument(
                "debug_image",
                default_value="false",
                description="Publish fusion/<camera>/debug_image. Needs "
                "publish_image:=true on cameras.launch.py.",
            ),
            Node(
                package="prop_fusion",
                executable="fusion",
                name="fusion",
                parameters=[
                    config,
                    {
                        "sensors_file": LaunchConfiguration("sensors_file"),
                        "debug_image": ParameterValue(
                            LaunchConfiguration("debug_image"),
                            value_type=bool,
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="prop_fusion",
                executable="object_map",
                name="object_map",
                parameters=[config],
                output="screen",
            ),
        ],
    )
