from launch import LaunchDescription 
from launch_ros.actions import Node

# calling a launch files from launch file with get_package_share_directory
# example at: keeeeths hw in sub bringup/launch

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
        ),

        Node(
            package='wheel_drivers',
            executable='joy_handsome',
            output='screen'
        )
    ])
