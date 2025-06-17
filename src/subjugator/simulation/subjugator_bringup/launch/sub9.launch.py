import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch every launch file in hw as a hardware group
    this_package = get_package_share_directory("subjugator_bringup")
    launch_dir = os.path.join(this_package, "launch")
    hw_dir = os.path.join(launch_dir, "hw")

    hw_actions = []
    for file_or_dir in os.listdir(hw_dir):
        if file_or_dir == "__pycache__":
            continue
        file_or_dir_path = os.path.join(hw_dir, file_or_dir)
        if os.path.isdir(file_or_dir_path):
            # If it's a directory, include all launch files in it
            for launch_file in os.listdir(file_or_dir_path):
                if launch_file.endswith(".launch.py"):
                    hw_actions.append(
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(file_or_dir_path, launch_file),
                            ),
                        ),
                    )
        elif file_or_dir.endswith(".launch.py"):
            # If it's a file, include it directly
            hw_actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(hw_dir, file_or_dir)),
                ),
            )

    hw_action = GroupAction(actions=hw_actions)

    # Launch subjugator_setup.launch.py
    subjugator_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_package, "launch", "subjugator_setup.launch.py"),
        ),
        launch_arguments={
            "gui": "false",
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(hw_action)
    ld.add_action(subjugator_setup)
    return ld
