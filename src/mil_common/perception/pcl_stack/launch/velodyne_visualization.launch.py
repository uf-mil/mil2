"""Velodyne driver + pcl_stack node for visualization and optional perception filters.

Usage::

    ros2 launch pcl_stack velodyne_visualization.launch.py

Examples::

    ros2 launch pcl_stack velodyne_visualization.launch.py use_input_cloud_filter:=true publish_dummy_tf:=true
    ros2 launch pcl_stack velodyne_visualization.launch.py use_object_detector:=true robot_frame:=base_link

``/persist_pcl`` is in the lidar frame when input filtering is off, and in ``global_frame`` (default ``enu``)
when ``use_input_cloud_filter`` is true (cloud is transformed and footprint/bounds filtered there).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    sensor_ip_arg = DeclareLaunchArgument(
        "sensor_ip",
        default_value="192.168.37.51",
        description="IP address of the Velodyne sensor.",
    )
    use_input_cloud_filter_arg = DeclareLaunchArgument(
        "use_input_cloud_filter",
        default_value="false",
        description="Transform to global_frame and run InputCloudFilter (robot footprint / optional bounds) before stacking.",
    )
    use_object_detector_arg = DeclareLaunchArgument(
        "use_object_detector",
        default_value="false",
        description="Run euclidean clustering on the persistent filtered mega-cloud each publish cycle (logs cluster count).",
    )
    robot_frame_arg = DeclareLaunchArgument(
        "robot_frame",
        default_value="base_link",
        description="Robot frame for CropBox removal when use_input_cloud_filter is true (TF: global_frame <- robot_frame).",
    )
    global_frame_arg = DeclareLaunchArgument(
        "global_frame",
        default_value="enu",
        description="Fixed world frame for transforming the cloud when use_input_cloud_filter is true; must match TF.",
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("velodyne"),
                "launch",
                "velodyne-all-nodes-VLP16-composed-launch.py",
            ),
        ),
        launch_arguments={
            "sensor_ip": LaunchConfiguration("sensor_ip"),
        }.items(),
    )

    pcl_stack_node = Node(
        package="pcl_stack",
        executable="pcl_stack_node",
        name="pcl_stack_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_input_cloud_filter": ParameterValue(
                    LaunchConfiguration("use_input_cloud_filter"),
                    value_type=bool,
                ),
                "use_object_detector": ParameterValue(
                    LaunchConfiguration("use_object_detector"),
                    value_type=bool,
                ),
                "robot_frame": LaunchConfiguration("robot_frame"),
                "global_frame": LaunchConfiguration("global_frame"),
            },
        ],
        remappings=[
            ("/velodyne_points", LaunchConfiguration("pointcloud_topic")),
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            LaunchConfiguration("global_frame"),
            "--child-frame-id",
            LaunchConfiguration("lidar_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_dummy_tf")),
    )

    return LaunchDescription(
        [
            pointcloud_topic_arg,
            lidar_frame_arg,
            publish_dummy_tf_arg,
            sensor_ip_arg,
            use_input_cloud_filter_arg,
            use_object_detector_arg,
            robot_frame_arg,
            global_frame_arg,
            velodyne_launch,
            static_tf,
            pcl_stack_node,
        ],
    )
