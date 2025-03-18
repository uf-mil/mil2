This package listens to the wrench topic and converts it to the thruster effort needed from each of the 8 thrusters on SubjuGator 9.
It relies on the thruster allocation matrix configured in the config yaml file.

Start node with `ros2 launch subjugator_thruster_manager thruster_manager.launch.py`
