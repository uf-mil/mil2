"""Perception for the maneuvers: scan conversion, filtering, clustering.

The simulated boat carries a flat single-ring scanner, so the scan is converted
to a point cloud before the pcd pipeline sees it. On the real boat the lidar
publishes a cloud directly: launch with scan_to_cloud:=false and point the
filter at the real topic.

The cluster size threshold is overridden here because the simulated lidar takes
one sample per degree, so a 0.5 m buoy at circling distance returns about five
points where pcd_params.yaml asks for twenty. This is a simulator artefact and
must NOT be copied back into pcd_params.yaml -- the real lidar has sixteen
rings and much finer horizontal resolution.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pcd_params = os.path.join(
        get_package_share_directory("pcd"),
        "config",
        "pcd_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scan_to_cloud",
                default_value="true",
                description="Convert a 2D scan to a cloud. True in simulation, false on the boat.",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/lidar/points",
                description="PointCloud2 the filter reads.",
            ),
            DeclareLaunchArgument(
                "cluster_min_points",
                default_value="3",
                description=(
                    "Simulator override. The simulated lidar samples once per degree, "
                    "so a buoy at circling distance is only a handful of points."
                ),
            ),
            Node(
                package="prop_maneuvers",
                executable="scan_to_cloud",
                name="scan_to_cloud",
                output="screen",
                parameters=[{"use_sim_time": True}],
                remappings=[
                    ("scan", "/lidar/scan"),
                    ("points", LaunchConfiguration("cloud_topic")),
                ],
                condition=IfCondition(LaunchConfiguration("scan_to_cloud")),
            ),
            Node(
                package="pcd",
                executable="pcl_filter_node",
                name="pcl_filter",
                output="screen",
                parameters=[
                    pcd_params,
                    {"use_sim_time": True},
                    {"input_topic": LaunchConfiguration("cloud_topic")},
                ],
            ),
            Node(
                package="pcd",
                executable="pcl_clustering_node",
                name="pcl_clustering",
                output="screen",
                parameters=[
                    pcd_params,
                    {"use_sim_time": True},
                    {"cluster_min_points": LaunchConfiguration("cluster_min_points")},
                ],
            ),
        ],
    )
