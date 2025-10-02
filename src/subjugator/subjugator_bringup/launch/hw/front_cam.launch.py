import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="front_cam",
                executable="front_cam",
                name="front_cam",
                parameters=[
                    os.path.join(
                        get_package_share_directory("front_cam"),
                    ),
                ],
                output="screen",
            ),
        ],
    )
