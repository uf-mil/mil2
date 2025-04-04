import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("reptile_bringup")
    pkg_project_description = get_package_share_directory("reptile_description")

    # Load the URDF file from "description" package
    xacro_file = os.path.join(pkg_project_description, "urdf", "reptile_rover.urdf.xacro")
    assert os.path.exists(xacro_file), "The reptile.urdf.xacro doesn't exist in " + str(
        xacro_file,
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    urdf_file = os.path.join(pkg_project_description, "urdf", "reptile_rover.urdf")
    with open(urdf_file, "w") as urdf:
        urdf.write(robot_desc)

    print(f"Successfully converted {xacro_file} to {urdf_file}")

    


    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'robot_description': robot_desc},
    #     ]
    # )

    # Visualize in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_project_bringup, "config", "reptile_rover.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz.",
            ),
            robot_state_publisher_node,
            # joint_state_publisher_node,
            rviz,
        ],
    )

