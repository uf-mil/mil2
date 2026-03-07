from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_description = get_package_share_path("navigator_description")
    # pkg_ros_gz_sim = get_package_share_path("ros_gz_sim")

    # Generate urdf from the xacro file
    xacro_file_arg = DeclareLaunchArgument(
        "xacro_file",
        default_value=pkg_project_description / "urdf" / "navigator.urdf.xacro",
        description="Path to the robot xacro file",
    )

    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="navigator",
        description="Model name to spawn",
    )

    model_name = LaunchConfiguration("model_name")
    urdf_out = pkg_project_description / "urdf" / "navigator.urdf"
    generate_urdf = ExecuteProcess(
        cmd=[
            "xacro",
            LaunchConfiguration("xacro_file"),
            "-o",
            urdf_out,
            ["model_name:=", model_name],
        ],
        output="screen",
    )

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
            urdf_out,
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

    return LaunchDescription(
        [xacro_file_arg, model_name_arg, generate_urdf, spawn_navigator],
    )
