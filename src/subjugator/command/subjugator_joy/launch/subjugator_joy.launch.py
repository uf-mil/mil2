from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy",
            ),
            Node(
                package="subjugator_joy",
                executable="joy_node",
                name="subjugator_joy",
            ),
        ],
    )
