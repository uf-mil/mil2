import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_capture_node = Node(
        package="object_classifier",
        executable="frame_capture_node.py",
        name="frame_capture_node",
        parameters=[{"camera_source": LaunchConfiguration("camera_source")}],
        condition=IfCondition(LaunchConfiguration("use_camera")),
        output="screen",
    )

    detection_node = Node(
        package="object_classifier",
        executable="detection_node.py",
        name="detection_node",
        parameters=[{"model_path": LaunchConfiguration("model_path")}],
        output="screen",
    )

    rqt_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        arguments=["/camera/detections/image"],
        condition=IfCondition(LaunchConfiguration("rqt")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_source", default_value=""),
            DeclareLaunchArgument(
                "model_path",
                default_value=os.path.expanduser("~/buoy_buoy10-13.pt"),
            ),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("rqt", default_value="true"),
            frame_capture_node,
            detection_node,
            rqt_node,
        ],
    )
