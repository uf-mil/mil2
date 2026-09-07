import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pcd_pkg_dir = get_package_share_directory("pcd")
    params_file = os.path.join(pcd_pkg_dir, "config", "pcd_params.yaml")

    filter_node = Node(
        package="pcd",
        executable="pcl_filter_node",
        name="pcl_filter",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([filter_node])
