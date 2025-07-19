from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="subjugator_mission_planner",
                executable="navigate_around_server",
                name="navigate_around_server",
                parameters=[],
            ),
            Node(
                package="subjugator_mission_planner",
                executable="movement_server",
                name="movement_server",
                parameters=[],
            ),
        ],
    )
