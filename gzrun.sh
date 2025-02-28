colcon build
source ./install/setup.bash
ros2 launch subjugator_bringup gazebo.launch.py "$@"
