# prop_gazebo

The simulated boat.

    ros2 launch prop_gazebo prop_sim.launch.py
    ros2 launch prop_gazebo prop_sim.launch.py control:=true mission:=square
    ros2 launch prop_gazebo prop_sim.launch.py rviz:=false \
        gz_args:="-s --headless-rendering"

Gazebo supplies the hull, the water and the sensors. Localization and control
are the same nodes the boat runs, so the only thing that changes between here
and the water is what publishes `/imu`, `/gps_raw` and `/lidar/scan`, and what
listens on `/thrusters/left` and `/thrusters/right`.

| Argument | Default | |
|---|---|---|
| `world` | `prop_lake.world` | World in `worlds/` |
| `control` | `false` | Run the mission, guidance and thruster manager |
| `mission` | `square` | Mission file for the controller |
| `control_delay` | `15.0` | Seconds to let localization settle before the controller starts |
| `rviz` | `true` | |
| `gz_args` | `--render-engine ogre2` | Extra Gazebo flags |

## What is in here

- `models/prop` is the boat: a 25 kg hull, an IMU where the real one is bolted
  (backwards, hence the yaw of pi), a GPS, a lidar, and hydrodynamic damping.
  Its centre of mass is ballasted below the waterline, because Gazebo's
  buoyancy pushes up through the middle of the hull and gives no righting
  moment of its own - without that the boat capsizes at the first nudge.
- `worlds/prop_lake.world` is open water with a few buoys for the lidar.
- `PropThrusters` applies the thrust. Gazebo ships a thruster system of its
  own, but it models a propeller and does not put out the newtons you ask it
  for, which is no use when the controller is solving for a known force.
- `sim_thrusters.py` stands in for the thruster driver: normalized effort in,
  newtons out, with the same heartbeat failsafe.
- `sim_sensors.py` puts covariances back onto the IMU and GPS. The Gazebo
  messages have nowhere to carry them, so the bridge hands over zeros and the
  EKFs would treat every measurement as perfect.

## Gotchas

- The GPS noise in `model.sdf` is in **degrees**, not metres: Gazebo applies it
  to latitude and longitude directly. `1.8e-07` is 2 cm.
- The world name is part of the Gazebo topic names, so `config/prop_bridge.yaml`
  is tied to `prop_lake.world`.
- The lidar scans flat, so it mostly misses the water, but a degree of pitch
  puts returns on it 20 to 30 m out. Buoys come back much nearer than that.
- `lidar_sim.launch.py` and `models/lidar_platform` are the older lidar-only
  setup, against the NaviGator RobotX world.
