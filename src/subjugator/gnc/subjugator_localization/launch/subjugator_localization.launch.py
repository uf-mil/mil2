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
                package="robot_localization",
                executable="ekf_node",
                name="subjugator_localization",
                parameters=[config_file],
                remappings=[
                    ("/reset", "/_reset"),  # given, but broken :/
                    ("/set_pose", "/subjugator_localization/set_pose"),
                    ("/enable", "/subjugator_localization/enable"),
                    ("/toggle", "/subjugator_localization/toggle"),
                ],
            ),
            Node(
                package="subjugator_localization",
                executable="reset_srv_node.py",
                name="reset_localization_service",
                output="screen",
            ),
            Node(
                package="subjugator_localization",
                executable="monitoring_subs.py",
                name="monitor_sensors",
            ),
        ],
    )
