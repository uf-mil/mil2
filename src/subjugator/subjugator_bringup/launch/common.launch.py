import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    # Args
    gui_cmd = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="whether to launch the gui",
    )

    xacro_file_arg = DeclareLaunchArgument(
        "xacro_file",
        default_value=pkg_share("subjugator_description", "urdf", "sub9.urdf.xacro"),
        description="Path to the robot xacro file",
    )

    # Expand xacro at runtime
    robot_desc = ParameterValue(
        Command(["xacro", " ", LaunchConfiguration("xacro_file")]),
        value_type=str,
    )

    # Write an on-disk URDF
    urdf_out = pkg_share("subjugator_description", "urdf", "sub9.urdf")
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
            {"robot_description": robot_desc},
        ],
    )

    # Visualize in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            pkg_share("subjugator_bringup", "config", "subjugator.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    thruster_manager = IncludeLaunchDescription(
        pkg_share(
            "subjugator_thruster_manager",
            "launch",
            "thruster_manager.launch.py",
        ),
    )

    localization = IncludeLaunchDescription(
        pkg_share(
            "subjugator_localization",
            "launch",
            "subjugator_localization.launch.py",
        ),
        launch_arguments={
            "params_file": pkg_share(
                "subjugator_localization",
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
        pkg_share("subjugator_controller", "launch", "pid_controller.launch.py"),
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

    new_depth = Node(
        package="new_depth_driver",
        executable="new_depth_driver",
        name="new_depth_driver",
        output="both",
    )

    # wrench_tuner = IncludeLaunchDescription(
    # pkg_share("subjugator_wrench_tuner", "launch", "wrench_tuner_launch.py")
    # )
    return LaunchDescription(
        [
            gui_cmd,
            xacro_file_arg,
            generate_urdf,
            robot_state_publisher_node,
            rviz,
            thruster_manager,
            localization,
            controller,
            path_planner,
            trajectory_planner,
            new_depth,
        ],
    )
