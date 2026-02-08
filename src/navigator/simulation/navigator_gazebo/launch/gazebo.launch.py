from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    # pkg_project_gazebo = get_package_share_path("navigator_gazebo")
    pkg_project_description = get_package_share_path("navigator_description")
    pkg_ros_gz_sim = get_package_share_path("ros_gz_sim")

    # Include model file path
    set_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH"),
            TextSubstitution(text=":"),
            get_package_share_path("wamv_description").parent,
            TextSubstitution(text=":"),
            get_package_share_path("wamv_gazebo").parent,
        ],
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim_world_arg = DeclareLaunchArgument("world", default_value="robotx2024.world")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_ros_gz_sim / "launch" / "gz_sim.launch.py"),
        # launch_arguments={
        #     "gz_args": [
        #         PathJoinSubstitution(
        #             [pkg_project_gazebo, "worlds" ,LaunchConfiguration("world")],
        #         ),
        #         " --render-engine",
        #         " ogre",
        #     ],
        # }.items(),
    )

    # Generate urdf from the xacro file
    xacro_file_arg = DeclareLaunchArgument(
        "xacro_file",
        default_value=pkg_project_description / "urdf" / "navigator.urdf.xacro",
        description="Path to the robot xacro file",
    )
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
            "navigator",
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
        [
            set_env,
            gz_sim_world_arg,
            gz_sim,
            xacro_file_arg,
            generate_urdf,
            spawn_navigator,
        ],
    )
