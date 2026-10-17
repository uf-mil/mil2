import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, NotEqualsSubstitution
from launch_ros.actions import Node

FRONT_CAM = "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0"
BACK_CAM = ""


def generate_launch_description():
    obj_class_pkg = get_package_share_directory("object_classifier")
    config_file = os.path.join(obj_class_pkg, "config", "cam.yaml")

    front_camera = LaunchConfiguration("front_camera")
    back_camera = LaunchConfiguration("back_camera")

    # cam.yaml is plain yaml shared with the non-ROS scripts, not a ROS params file
    def camera_node(name, source, condition=None):
        return Node(
            package="object_classifier",
            executable="frame_capture_node.py",
            name="frame_capture",
            namespace=name,
            parameters=[
                {
                    "config_file": config_file,
                    "camera_source": source,
                    "frame_id": name,
                },
            ],
            condition=condition,
            output="screen",
        )

    rqt_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "front_camera",
                default_value=FRONT_CAM,
                description="v4l device path or index",
            ),
            DeclareLaunchArgument(
                "back_camera",
                default_value=BACK_CAM,
                description="v4l device path or index; empty skips the back camera",
            ),
            camera_node("front_cam", front_camera),
            camera_node(
                "back_cam",
                back_camera,
                IfCondition(NotEqualsSubstitution(back_camera, "")),
            ),
            rqt_node,
        ],
    )
