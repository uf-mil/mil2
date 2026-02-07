#### want to publish these messages yourself from the terminal to debug??

##### wrench

ros2 topic pub -r 10 /cmd_wrench geometry_msgs/msg/Wrench "{
  force: {x: 1.0, y: 0.0, z: -2.0},
  torque: {x: 0.0, y: 0.0, z: 0.5}
}"

##### effort

ros2 topic pub --once /thruster_efforts subjugator_msgs/msg/ThrusterEfforts "{
  thrust_flh: 0.2,
  thrust_frh: 0.2,
  thrust_blh: 0.1,
  thrust_brh: 0.1,
  thrust_flv: 0.0,
  thrust_frv: 0.0,
  thrust_blv: 0.0,
  thrust_brv: 0.0
}"
