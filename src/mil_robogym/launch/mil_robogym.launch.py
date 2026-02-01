from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mil_robogym",
                executable="mil_robogym",
                name="mil_robogym",
                output="screen",
            ),
        ],
    )
