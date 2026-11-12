import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    yolo_launch = os.path.join(
        get_package_share_directory("yolo_bringup"),
        "launch",
        "yolo.launch.py",
    )

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
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "undistorted_image_topic": LaunchConfiguration("undistorted_image_topic"),
                "yolo_detections_topic": LaunchConfiguration("yolo_detections_topic"),
            },
        ],
        output="screen",
    )

    # Reuse the already-built Subjugator yolo_ros stack (ONNX on CPU).
    yolo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yolo_launch),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "input_image_topic": LaunchConfiguration("undistorted_image_topic"),
            "image_reliability": LaunchConfiguration("image_reliability"),
            "namespace": LaunchConfiguration("namespace"),
            "device": LaunchConfiguration("device"),
            "enable": LaunchConfiguration("enable"),
            "half": LaunchConfiguration("half"),
            "threshold": LaunchConfiguration("threshold"),
            "tracker": LaunchConfiguration("tracker"),
            "use_tracking": LaunchConfiguration("use_tracking"),
            "use_debug": LaunchConfiguration("use_debug"),
        }.items(),
    )

    rqt_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        arguments=["/yolo/dbg_image"],
        condition=IfCondition(LaunchConfiguration("rqt")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_source", default_value=""),
            DeclareLaunchArgument(
                "model",
                default_value="/prop/prop_vision/models/best.onnx",
                description="Path to YOLO ONNX weights (CPU / ONNX Runtime)",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/cam/image/raw",
            ),
            DeclareLaunchArgument(
                "undistorted_image_topic",
                default_value="/cam/image/undistorted",
            ),
            DeclareLaunchArgument(
                "yolo_detections_topic",
                default_value="/yolo/tracking",
            ),
            DeclareLaunchArgument("image_reliability", default_value="1"),
            DeclareLaunchArgument("namespace", default_value="yolo"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("enable", default_value="True"),
            DeclareLaunchArgument("half", default_value="False"),
            DeclareLaunchArgument("threshold", default_value="0.5"),
            DeclareLaunchArgument("tracker", default_value="bytetrack.yaml"),
            DeclareLaunchArgument("use_tracking", default_value="True"),
            DeclareLaunchArgument("use_debug", default_value="True"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("rqt", default_value="true"),
            frame_capture_node,
            detection_node,
            yolo_node,
            rqt_node,
        ],
    )
