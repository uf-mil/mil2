"""
Label the lidar's tracks with what the camera sees.

    ros2 launch prop_fusion fusion.launch.py
    ros2 launch prop_fusion fusion.launch.py debug_image:=true

The detections have to be in the same pixels as camera_info describes: run
YOLO on the rectified image and pass that image's camera_info. Their header's
frame_id is the frame the tracks are projected into, so it has to be the
camera's optical frame and has to be in TF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detections",
                default_value="/yolo/detections",
                description="yolo_msgs/DetectionArray, in rectified pixels.",
            ),
            DeclareLaunchArgument(
                "camera_info",
                default_value="/camera/camera_info",
                description="CameraInfo of the image the detections came from.",
            ),
            DeclareLaunchArgument(
                "image",
                default_value="/camera/image_rect",
                description="The image the detections came from, for debug_image.",
            ),
            DeclareLaunchArgument(
                "debug_image",
                default_value="false",
                description="Publish fusion/debug_image with the tracks drawn on.",
            ),
            Node(
                package="prop_fusion",
                executable="fusion",
                name="fusion",
                parameters=[
                    os.path.join(
                        get_package_share_directory("prop_fusion"),
                        "config",
                        "fusion.yaml",
                    ),
                    {"debug_image": LaunchConfiguration("debug_image")},
                ],
                remappings=[
                    ("detections", LaunchConfiguration("detections")),
                    ("camera_info", LaunchConfiguration("camera_info")),
                    ("image", LaunchConfiguration("image")),
                ],
                output="screen",
            ),
        ],
    )
