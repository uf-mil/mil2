"""Launch pcodar (pcl_stack_node) for just he visualization of the lidar Scan.

Usage:
    cb
    ros2 launch pcl_stack pcodar_visualization.launch.py

Optional arguments::

    ros2 launch pcl_stack pcodar_visualization.launch.py \\
        pointcloud_topic:=/your/lidar/points \\
        lidar_frame:=velodyne \\
        publish_dummy_tf:=true

Subscribes to a PointCloud2 (default ``/velodyne/points``) and publishes
intensity-filtered clouds on ``/persist_pcl`` in the same frame as the input.
In RViz, set fixed frame to that lidar frame when displaying the cloud.

``publish_dummy_tf`` publishes identity ``enu`` -> ``lidar_frame`` if needed;
it defaults to false to avoid TF conflicts.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pcl_stack")
    params_file = os.path.join(pkg_share, "launch", "pcodar.yaml")

    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="/velodyne/points",
        description="Input PointCloud2 topic (remapped from the node's /velodyne_points subscription).",
    )
    lidar_frame_arg = DeclareLaunchArgument(
        "lidar_frame",
        default_value="velodyne",
        description="Frame id of the input cloud; must match PointCloud2 header. "
        "Used only when publish_dummy_tf is true.",
    )
    publish_dummy_tf_arg = DeclareLaunchArgument(
        "publish_dummy_tf",
        default_value="false",
        description="If true, publish identity static TF: parent enu, child lidar_frame.",
    )

    pcl_stack_node = Node(
        package="pcl_stack",
        executable="pcl_stack_node",
        name="pcl_stack_node",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("/velodyne_points", LaunchConfiguration("pointcloud_topic")),
        ],
    )

    # x y z yaw pitch roll parent_frame child_frame
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "enu",
            LaunchConfiguration("lidar_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_dummy_tf")),
    )

    return LaunchDescription(
        [
            pointcloud_topic_arg,
            lidar_frame_arg,
            publish_dummy_tf_arg,
            static_tf,
            pcl_stack_node,
        ],
    )
