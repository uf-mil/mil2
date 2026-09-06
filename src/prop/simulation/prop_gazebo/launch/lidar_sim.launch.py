import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from xacro import process_file


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def spawn_robot(context, *args, **kwargs):
    model_name = LaunchConfiguration('model_name').perform(context)
    xacro_file = pkg_share('prop_gazebo', 'urdf', 'lidar_platform.urdf.xacro')
    robot_desc = process_file(xacro_file).toxml()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_desc},
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name',
                model_name,
                '-string',
                robot_desc,
                '-x',
                '0.0',
                '-y',
                '0.0',
                '-z',
                '2.0',
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='robotx_2024.world',
        description='World file to load in Gazebo.',
    )

    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='lidar_platform',
        description='Name of the spawned lidar platform model.',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share('ros_gz_sim', 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution(
                    [
                        pkg_share('navigator_gazebo'),
                        'worlds',
                        LaunchConfiguration('world'),
                    ],
                ),
                ' --render-engine',
                ' ogre2',
            ],
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': pkg_share('prop_gazebo', 'config', 'lidar_bridge.yaml'),
            },
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            world_arg,
            model_name_arg,
            SetParameter('use_sim_time', True),
            gz_sim,
            OpaqueFunction(function=spawn_robot),
            bridge,
        ],
    )
