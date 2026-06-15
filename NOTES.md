# navtube ROS2 migration notes (#455)

## Goal
Run the vectornav IMU driver on the navtube Raspberry Pi and communicate over
ROS2, replacing the current socat / TCP serial-bridge setup.

## Current architecture (before)
- The VectorNav IMU is in the navtube. Its serial line is exposed over the
  network by a serial-to-TCP device server at 192.168.37.61:10001.
- On the Jetson, the `imu-socat` alias (scripts/setup.bash, ~line 31) runs socat
  to create a virtual serial port /dev/ttyV0 forwarding to that TCP socket.
- joe_shmore.py is a watchdog that keeps /dev/ttyV0 primed (baud 921600).
- The vectornav ROS2 node opens /dev/ttyV0 and publishes IMU topics.

Data path: IMU -> serial/TCP bridge (.61:10001) -> Jetson socat -> /dev/ttyV0
-> vectornav node -> ROS2 topics

## Target architecture (after)
- vectornav runs ON the navtube Pi, reading the IMU's real serial device.
- The Pi publishes IMU topics; CycloneDDS carries them to the Jetson and other
  subscribers automatically (same network + ROS_DOMAIN_ID).
- socat, the TCP bridge, and joe_shmore are no longer needed.

Data path: IMU -> Pi (vectornav node) -> ROS2 DDS over network -> subscribers

## Key files
- scripts/setup.bash — has the imu-socat alias to remove; also sets ROS Jazzy
  and the CycloneDDS config.
- joe_shmore.py — socat watchdog, to remove.
- cyclone.xml — CycloneDDS discovery config.
- src/subjugator/drivers/vectornav (submodule -> uf-mil/vectornav) — the driver.
  - vectornav/config/vectornav.yaml line 5: port: "/dev/ttyV0"  <- KEY change;
    repoint to the Pi's real device (likely /dev/ttyUSB0).
  - vectornav/launch/vectornav.launch.py — launch entry point; confirms which
    config file is loaded.

## Dependencies / blockers
- #454: ROS2 on the navtube Pi via RoboStack/pixi. Must land before vectornav
  can run on the Pi.
- Needs hardware power-on to test (lab; possibly remote via VPN/Tailscale).
- Confirm the IMU's real device path on the Pi and the model (vn100 vs vn200).

## Open questions for wingdeans
- Is #454 done, and who owns it?
- What's the IMU's serial device path on the Pi? Is there a udev rule?
- Anything in the systemd units (coming to navigation-tube) I should mirror?

## Plan / checklist
- [ ] Read systemd units once pushed to navigation-tube
- [ ] Confirm #454 status
- [ ] Get the workspace building + sourcing on a ROS2 machine
- [ ] Repoint vectornav.yaml port to the Pi's real device
- [ ] Run vectornav on the Pi; verify with `ros2 topic echo`
- [ ] Configure cross-machine DDS; verify the Jetson sees the topics
- [ ] Add a systemd unit for boot autostart on the Pi
- [ ] Remove imu-socat alias + joe_shmore.py; update docs
- [ ] Test end-to-end in lab; compare against the old path