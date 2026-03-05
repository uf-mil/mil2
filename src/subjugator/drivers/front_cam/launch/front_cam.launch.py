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
                        "camera-id": "/dev/v4l/by-id/usb-e-con_Systems_See3CAM_CU20_06040000-video-index0",
                        "camera-topic": "front_cam/image_raw",
                    },
                ],
                output="screen",
            ),
        ],
    )
