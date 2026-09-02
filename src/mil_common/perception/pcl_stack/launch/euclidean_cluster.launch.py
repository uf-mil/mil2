"""Bare-bones Euclidean clustering for Gazebo / sim lidar.

Subscribes to the navigator sim PointCloud2 topic and runs the PCL
EuclideanClusterExtraction pipeline from:
https://pointclouds.org/documentation/tutorials/cluster_extraction.html

Example (sim already publishing /velodyne/points)::

    ros2 launch pcl_stack euclidean_cluster.launch.py

RViz: add PointCloud2 on ``/clusters_cloud`` and MarkerArray on ``/cluster_markers``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="/velodyne/points",
        description="Sim / hardware PointCloud2 topic.",
    )
    voxel_leaf_size_arg = DeclareLaunchArgument(
        "voxel_leaf_size",
        default_value="0.1",
        description="VoxelGrid leaf size in meters (0 disables downsampling).",
    )
    remove_planes_arg = DeclareLaunchArgument(
        "remove_planes",
        default_value="true",
        description="Repeatedly remove dominant planes before clustering (tutorial step).",
    )
    cluster_tolerance_arg = DeclareLaunchArgument(
        "cluster_tolerance",
        default_value="0.5",
        description="Max distance between points in a cluster (meters).",
    )
    cluster_min_points_arg = DeclareLaunchArgument(
        "cluster_min_points",
        default_value="20",
        description="Minimum points per cluster.",
    )
    cluster_max_points_arg = DeclareLaunchArgument(
        "cluster_max_points",
        default_value="25000",
        description="Maximum points per cluster.",
    )

    cluster_node = Node(
        package="pcl_stack",
        executable="euclidean_cluster_node",
        name="euclidean_cluster_node",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("pointcloud_topic"),
                "voxel_leaf_size": ParameterValue(
                    LaunchConfiguration("voxel_leaf_size"),
                    value_type=float,
                ),
                "remove_planes": ParameterValue(
                    LaunchConfiguration("remove_planes"),
                    value_type=bool,
                ),
                "cluster_tolerance": ParameterValue(
                    LaunchConfiguration("cluster_tolerance"),
                    value_type=float,
                ),
                "cluster_min_points": ParameterValue(
                    LaunchConfiguration("cluster_min_points"),
                    value_type=int,
                ),
                "cluster_max_points": ParameterValue(
                    LaunchConfiguration("cluster_max_points"),
                    value_type=int,
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            pointcloud_topic_arg,
            voxel_leaf_size_arg,
            remove_planes_arg,
            cluster_tolerance_arg,
            cluster_min_points_arg,
            cluster_max_points_arg,
            cluster_node,
        ],
    )
