This package listens to the wrench topic and converts it to the thruster effort needed from each of the 8 thrusters on SubjuGator 9.
It relies on the thruster allocation matrix configured in the config yaml file.

The desired wrench is allocated to per-thruster forces (Newtons) via the pseudo-inverse of the
allocation matrix, then converted to normalized effort `[-1, 1]` through the measured thrust curve in
`include/subjugator_thruster_manager/lut.h`. That curve is nonlinear and asymmetric (a thruster makes
more force forward than reverse), so this replaces the old single-slope `max_force_pos`/`max_force_neg`
scaling. `forward_to_sim` applies the same curve in reverse (`force_from_effort`) so the simulator and
the real vehicle share one thrust model. Saturation against `thruster_cap` is applied in force units
before the curve so the wrench direction is preserved.

### The thrust curve (`lut.h`)

The table is the measured T200 curve transcribed from the thruster board ("purple board") and
converted from kgf to Newtons. It holds 201 samples evenly spaced in effort from `-1.0` (full
reverse, `-39.9130655` N) to `+1.0` (full forward, `+51.4849125` N), so full forward is ~29% stronger
than full reverse. Efforts within `|effort| <= 0.07` are the deadband and produce 0 N. The properties
the rest of the system depends on (endpoints, monotonicity, deadband, force/effort round-trip,
asymmetry) are enforced by `test/test_lut.cpp`.

Two assumptions are not yet verified and should be before trusting this on hardware:

- **Voltage:** the samples correspond to a single supply voltage (assumed ~16V). The sub runs
  ~15.5-16V, which shifts max thrust a few percent. Voltage-aware curve selection is deferred to
  separate follow-up work, since it first needs the board to report pack voltage (no such telemetry
  exists today).
- **Direction:** positive effort is assumed to be each thruster's stronger (forward) direction. The
  T200s use mixed CW/CCW props, so confirm a positive command on every thruster lands on the strong
  side of the curve, not the weak side — otherwise the asymmetry is applied backwards on those
  thrusters.

Start node with `ros2 launch subjugator_thruster_manager thruster_manager.launch.py`
