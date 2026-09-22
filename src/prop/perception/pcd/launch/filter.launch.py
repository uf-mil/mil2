"""
Lidar perception: filter the cloud, cluster it, track the clusters.

    ros2 launch pcd filter.launch.py
    ros2 launch pcd filter.launch.py use_sim_time:=true input_topic:=/lidar/points

The defaults are the boat: wall clock, and the Velodyne driver's own topic.
Gazebo needs both arguments overridden - it runs on sim time, and the bridge in
prop_gazebo/config/prop_bridge.yaml lands the cloud on /lidar/points.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pcd_pkg_dir = get_package_share_directory("pcd")
    params_file = os.path.join(pcd_pkg_dir, "config", "pcd_params.yaml")

    # Overrides the yaml, which is written for the boat.
    input_topic = {"input_topic": LaunchConfiguration("input_topic")}

    # Publish the static transform base_link → velodyne.
    # This matches the base_to_lidar joint in prop_localization/urdf/prop.urdf
    # Hardware driver published the lidar data in the velodyne frame, so we need to transform it to the base_link frame for processing.
    velodyne_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="velodyne_to_base_link_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.45",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "velodyne",
        ],
        output="screen",
    )

    filter_node = Node(
        package="pcd",
        executable="pcl_filter_node",
        name="pcl_filter",
        parameters=[params_file, input_topic],
        output="screen",
    )

    clustering_node = Node(
        package="pcd",
        executable="pcl_clustering_node",
        name="pcl_clustering",
        parameters=[params_file],
        output="screen",
    )

    tracker_node = Node(
        package="pcd",
        executable="pcl_tracker_node",
        name="pcl_tracker",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="True under Gazebo. TF lookups are made at the "
                "stamp on the incoming cloud, so a node on the wrong clock "
                "drops every detection it is handed.",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/velodyne_points",
                description="Raw PointCloud2 to filter. /lidar/points in "
                "simulation.",
            ),
            # Only set when asked. Left alone, the clock is whatever the bringup
            # that included this file chose, and an unconditional false here
            # would override it for every node launched after this one.
            SetParameter(
                "use_sim_time",
                True,
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            velodyne_tf,
            filter_node,
            clustering_node,
            tracker_node,
        ],
    )
