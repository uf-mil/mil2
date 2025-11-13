from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="subjugator_sensor_monitoring",
                executable="redundancy_check.py",
                name="redundancy_check_node",
                output="screen",
            ),
        ],
    )
