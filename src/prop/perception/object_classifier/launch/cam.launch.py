from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_capture_node = Node(
        package="object_classifier",
        executable="frame_capture_node.py",
        name="FrameCapture",
        parameters=[{"camera_source": LaunchConfiguration("camera_source")}],
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
            DeclareLaunchArgument("camera_source", default_value=""),
            frame_capture_node,
            rqt_node,
        ],
    )
