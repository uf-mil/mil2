from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def static_tf(parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            str(xyz[0]),
            str(xyz[1]),
            str(xyz[2]),
            str(rpy[0]),
            str(rpy[1]),
            str(rpy[2]),
            parent,
            child,
        ],
        output="screen",
    )


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
        cmd=["xacro", LaunchConfiguration("xacro_file"), "-o", urdf_out],
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

    # Publish a tf to resolve the extra model_name at the beginning of frame
    # Publish tfs to remap sensor frames
    base_frame = [model_name, TextSubstitution(text="/wamv/base_link")]
    lidar_frame = [*base_frame, "/velodyne_sensor"]
    front_left_camera_frame = [*base_frame, "/front_left_cam_sensor"]
    front_right_camera_frame = [*base_frame, "/front_right_cam_sensor"]
    tf_nodes = [
        static_tf("wamv/base_link", base_frame, (0, 0, 0), (0, 0, 0)),
        static_tf("velodyne", lidar_frame, (0, 0, 0), (0, 0, 0)),
        static_tf(
            "wamv/front_left_cam_link_optical",
            front_left_camera_frame,
            (0, 0, 0),
            (0, 0, 0),
        ),
        static_tf(
            "wamv/front_right_cam_link_optical",
            front_right_camera_frame,
            (0, 0, 0),
            (0, 0, 0),
        ),
    ]

    return LaunchDescription(
        [
            xacro_file_arg,
            model_name_arg,
            generate_urdf,
            spawn_navigator,
            *tf_nodes,
        ],
    )
