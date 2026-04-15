from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_description = get_package_share_path("navigator_description")
    # pkg_ros_gz_sim = get_package_share_path("ros_gz_sim")

    # Generate urdf from the xacro file
    model_file_arg = DeclareLaunchArgument(
        "model_file",
        default_value=pkg_project_description / "urdf" / "navigator.urdf",
        description="Path to the robot model file (.urdf or .sdf)",
    )

    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="navigator",
        description="Model name to spawn",
    )

    model_name = LaunchConfiguration("model_name")
    model_file = LaunchConfiguration("model_file")

    # Spawn the navigator
    navigator_x = "0.0"
    navigator_y = "0.0"
    navigator_z = "0.0"
    navigator_yaw = "0.0"
    spawn_navigator = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            model_name,
            "-file",
            model_file,
            "-x",
            navigator_x,
            "-y",
            navigator_y,
            "-z",
            navigator_z,
            "-Y",
            navigator_yaw,
        ],
        output="screen",
    )

    Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/velodyne/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    return LaunchDescription(
        [model_file_arg, model_name_arg, spawn_navigator],
    )
