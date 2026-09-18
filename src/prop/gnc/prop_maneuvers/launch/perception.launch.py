"""
Perception for the maneuvers: scan conversion, filtering, clustering.

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
    """Launch scan_to_cloud (sim only), pcl_filter, and pcl_clustering, wired together."""
    maneuvers_share = get_package_share_directory("prop_maneuvers")
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
                "use_sim_time",
                default_value="true",
                description="Use the /clock topic. True in simulation, false on the real boat.",
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
            DeclareLaunchArgument(
                "blind_spots",
                default_value="false",
                description="Fake the real boat's blind spots by masking the scan.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/lidar/scan",
                description="Set to /lidar/scan_masked when blind_spots is true.",
            ),
            Node(
                package="prop_maneuvers",
                executable="blind_spot_mask",
                name="blind_spot_mask",
                output="screen",
                parameters=[
                    os.path.join(maneuvers_share, "config", "maneuvers.yaml"),
                    {"use_sim_time": True},
                ],
                remappings=[
                    ("scan", "/lidar/scan"),
                    ("masked_scan", "/lidar/scan_masked"),
                ],
                condition=IfCondition(LaunchConfiguration("blind_spots")),
            ),
            Node(
                package="prop_maneuvers",
                executable="scan_to_cloud",
                name="scan_to_cloud",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=[
                    ("scan", LaunchConfiguration("scan_topic")),
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
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
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
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"cluster_min_points": LaunchConfiguration("cluster_min_points")},
                ],
            ),
        ],
    )
