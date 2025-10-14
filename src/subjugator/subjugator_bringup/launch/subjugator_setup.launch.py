import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("subjugator_bringup")
    pkg_project_description = get_package_share_directory("subjugator_description")
    pkg_thruster_manager = get_package_share_directory("subjugator_thruster_manager")
    pkg_localization = get_package_share_directory("subjugator_localization")
    pkg_controller = get_package_share_directory("subjugator_controller")

    # Args
    gui_cmd = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="whether to launch the gui",
    )

    xacro_file_arg = DeclareLaunchArgument(
        "xacro_file",
        default_value=PathJoinSubstitution(
            [pkg_project_description, "urdf", "sub9.urdf.xacro"],
        ),
        description="Path to the robot xacro file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use sim clock (true for sim, false for real)",
    )

    # Expand xacro at runtime
    robot_desc = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", LaunchConfiguration("xacro_file")]),
        value_type=str,
    )

    # Write an on-disk URDF
    urdf_out = os.path.join(pkg_project_description, "urdf", "sub9.urdf")
    generate_urdf = ExecuteProcess(
        cmd=["xacro", LaunchConfiguration("xacro_file"), "-o", urdf_out],
        output="screen",
    )

    # # Convert URDF to SDF using Gazebo's gz tool
    # sdf_file = os.path.join(pkg_project_description, 'urdf', 'sub9.sdf')
    # try:
    #     subprocess.run(['gz', 'sdf', '-p', urdf_file], check=True, stdout=open(sdf_file, 'w'))
    #     print(f"Successfully converted {urdf_file} to {sdf_file}")
    # except subprocess.CalledProcessError as e:
    #     print(f"Error converting URDF to SDF: {e}")

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": robot_desc},
        ],
    )

    # Visualize in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_project_bringup, "config", "subjugator.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_thruster_manager, "launch", "thruster_manager.launch.py"),
        ),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_localization,
                "launch",
                "subjugator_localization.launch.py",
            ),
        ),
        launch_arguments={
            "params_file": os.path.join(
                pkg_localization,
                "config",
                "localization_parameters.yaml",
            ),
        }.items(),
    )

    # depth_to_pose = Node(
    # package="subjugator_localization",
    # executable="depth_to_pose_node.py",
    # name="depth_to_pose",
    # output="screen",
    # )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_controller, "launch", "pid_controller.launch.py"),
        ),
    )

    path_planner = Node(
        package="subjugator_path_planner",
        executable="subjugator_path_planner",
        name="subjugator_path_planner",
        output="both",
    )

    trajectory_planner = Node(
        package="subjugator_trajectory_planner",
        executable="trajectory_planner",
        name="subjugator_trajectory_planner",
        output="both",
    )

    # wrench_tuner = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(
    # os.path.join(
    # get_package_share_directory("subjugator_wrench_tuner"),
    # "launch",
    # "wrench_tuner_launch.py",
    # ),
    # ),
    # )
    return LaunchDescription(
        [
            gui_cmd,
            xacro_file_arg,
            use_sim_time_arg,
            generate_urdf,
            robot_state_publisher_node,
            # joint_state_publisher_node,
            rviz,
            thruster_manager,
            localization,
            controller,
            path_planner,
            trajectory_planner,
        ],
    )
