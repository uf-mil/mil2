"""Launch the VLP-16 velodyne driver against the physical lidar on the boat."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('prop_gazebo'),
        'config',
        'VLP16-velodyne_driver_node-params.yaml',
    )

    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='both',
        parameters=[params],
    )

    return LaunchDescription(
        [
            velodyne_driver,
            # Shut down the launch when the driver exits (e.g. lidar disconnected).
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=velodyne_driver,
                    on_exit=[EmitEvent(event=Shutdown())],
                ),
            ),
        ],
    )
