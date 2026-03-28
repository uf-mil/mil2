# joe handsome is the goat

alr so first how to get the thingy started:

`ros2 launch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@192.168.37.151:14550`

```
192.168.37.151 should be the Pi's ip address, the ports should lowkey never change
```

then run `ros2 topic list` to see a list of topics

But joe, what do those topics do??

See this: i couldn't find anything good luck

### commonly used topics:

Where am I?
`/mavros/local_position/pose`
`/mavros/local_position/odom`
`/mavros/global_position/global`
/mavros/global_position/rel_alt     # altitude above home
/mavros/global_position/compass_hdg # heading

Am i armed rn?
/mavros/state

Send pose here and it'll go to it maybe
/mavros/setpoint_position/local
/mavros/setpoint_position/global
/mavros/setpoint_velocity/cmd_vel

Don't use these but they exist:
/mavros/setpoint_raw/local
/mavros/setpoint_raw/global
/mavros/setpoint_raw/attitude

.

🚨 Critical gotcha (this trips everyone)

To actually control the drone using setpoints:

You MUST:
Switch to OFFBOARD mode
Continuously publish setpoints (~10–50 Hz)

If you don’t:

The drone ignores your commands
Or drops out of OFFBOARD immediately

This behavior comes from autopilots like PX4.

