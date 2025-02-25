import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="subjugator_thruster_manager",
                namespace="subjugator",
                executable="thruster_manager",
                name="thruster_manager",
                parameters=[
                    os.path.join(
                        get_package_share_directory("subjugator_thruster_manager"),
                        "config",
                        "thruster_manager.yaml",
                    ),
                ],
                output="screen",
            ),
        ],
    )
