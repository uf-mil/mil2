import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("subjugator_localization"),
        "config",
        "localization_parameters.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="subjugator_localization",
                executable="ekf_node",
                name="subjugator_localization",
                parameters=[config_file],
            ),
        ],
    )
