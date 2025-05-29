from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('subjugator_wrench_tuner'),
        'config',
        'wrench_tuner_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='subjugator_wrench_tuner',
            executable='wrench_tuner',
            name='wrench_tuner',
            output='screen',
            parameters=[config_file]
        )
    ])
