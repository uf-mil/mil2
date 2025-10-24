from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch online_bagger node with topics list
            Node(
                package="online_bagger",
                executable="online_bagger_server",
                output="screen",
                parameters=[{"topics": ["rand_int", "rand_float", "rand_str"]}],
            ),
        ],
    )
