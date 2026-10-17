import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
    obj_class_pkg = get_package_share_directory("object_classifier")
    params_file = os.path.join(obj_class_pkg, "config", "cam.yaml")

    frame_capture_node = Node(
        package="object_classifier",
        executable="frame_capture_node",
        name="FrameCapture",
        parameters=[params_file],
        output="screen"
    )

    rqt_node = Node (
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        output="screen",
    )

    return LaunchDescription([frame_capture_node, rqt_node])