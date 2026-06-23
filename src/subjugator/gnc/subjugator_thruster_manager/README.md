This package listens to the wrench topic and converts it to the thruster effort needed from each of the 8 thrusters on SubjuGator 9.
It relies on the thruster allocation matrix configured in the config yaml file.

The desired wrench is allocated to per-thruster forces (Newtons) via the pseudo-inverse of the
allocation matrix, then converted to normalized effort `[-1, 1]` through the measured thrust curve in
`include/subjugator_thruster_manager/lut.h`. That curve is nonlinear and asymmetric (a thruster makes
more force forward than reverse), so this replaces the old single-slope `max_force_pos`/`max_force_neg`
scaling. `forward_to_sim` applies the same curve in reverse (`force_from_effort`) so the simulator and
the real vehicle share one thrust model. Saturation against `thruster_cap` is applied in force units
before the curve so the wrench direction is preserved.

Start node with `ros2 launch subjugator_thruster_manager thruster_manager.launch.py`
