import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="subjugator_controller",
                executable="subjugator_controller",
                name="pid_controller",
                parameters=[
                    os.path.join(
                        get_package_share_directory("subjugator_controller"),
                        "config",
                        "pid_controller.yaml",
                    ),
                ],
                output="screen",
            ),
        ],
    )
