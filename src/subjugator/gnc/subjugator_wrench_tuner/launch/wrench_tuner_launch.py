from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='subjugator_wrench_tuner',
            executable='wrench_tuner',
            name='wrench_tuner',
            parameters=['config/wrench_tuner_params.yaml'],  
            output='screen'
        )
    ])
