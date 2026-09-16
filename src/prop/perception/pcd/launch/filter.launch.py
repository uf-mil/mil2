import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pcd_pkg_dir = get_package_share_directory("pcd")
    params_file = os.path.join(pcd_pkg_dir, "config", "pcd_params.yaml")

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
        parameters=[params_file],
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

    return LaunchDescription([velodyne_tf, filter_node, clustering_node, tracker_node])
