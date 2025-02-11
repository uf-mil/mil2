from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = [{"name": "alex"}]
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                namespace="subjugator_localization",
                executable="ekf_localization_node",
                name="ekf_node",
                parameters=parameters,
            ),
        ],
    )
