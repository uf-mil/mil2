from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="navigator_kill_board",
                executable="heartbeat_server_node",
                name="heartbeat_server",
            ),
            Node(
                package="navigator_kill_board",
                executable="simulated_kill_board_node",
                name="simulated_kill_board",
            ),
        ],
    )
