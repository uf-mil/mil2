from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="front_cam",
                executable="front_cam",
                name="front_cam",
                parameters=[
                    {
                        "camera-id": "/dev/video0",
                        "camera-topic": "front_cam/image_raw",
                    },
                ],
                output="screen",
            ),
        ],
    )
