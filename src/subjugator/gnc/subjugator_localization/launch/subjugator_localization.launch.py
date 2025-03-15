import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('subjugator_localization'),
        'config',
        'parameter_definitions.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            namespace='subjugator_localization',
            parameters=[config_file],
        )
    ])