#!/usr/bin/env python3
"""
Thruster driver for the prop boat. Runs on the Pi, against the Navigator board.

    in  thrusters/left, thrusters/right  std_msgs/Float32, -1.0 to 1.0
        thrusters/heartbeat              std_msgs/Empty

Efforts are normalized: -1.0 is full reverse, 1.0 is full forward. They are sent
to the ESCs as servo pulses, which is what a BlueRobotics T200 expects.

Two seconds without a heartbeat zeroes both thrusters and ignores further
efforts until a heartbeat arrives, so a controller that dies (or a radio link
that drops) stops the boat instead of leaving it running.

    ros2 topic pub /thrusters/heartbeat std_msgs/msg/Empty
"""

import bluerobotics_navigator as nav
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32

# PWM channels on the Navigator board that the ESCs are wired to.
LEFT_CHANNEL = 2
RIGHT_CHANNEL = 0

PWM_FREQ_HZ = 500
PWM_COUNTS = 4096
NEUTRAL_US = 1500  # pulse width that stops the thruster
RANGE_US = 400  # pulse width added at full forward effort

HEARTBEAT_TIMEOUT = 2.0  # seconds of silence before thrust is cut


def counts(effort: float) -> int:
    """Normalized effort to a PWM count at PWM_FREQ_HZ."""
    effort = max(-1.0, min(1.0, effort))
    pulse_us = NEUTRAL_US + effort * RANGE_US
    return round(PWM_COUNTS * pulse_us / (1_000_000 / PWM_FREQ_HZ))


class Thrusters(Node):
    def __init__(self):
        super().__init__("thrusters")
        self.dead = False

        self.create_subscription(
            Float32,
            "thrusters/left",
            lambda msg: self.drive(LEFT_CHANNEL, msg.data),
            10,
        )
        self.create_subscription(
            Float32,
            "thrusters/right",
            lambda msg: self.drive(RIGHT_CHANNEL, msg.data),
            10,
        )
        self.create_subscription(
            Empty,
            "thrusters/heartbeat",
            self.heartbeat,
            10,
        )

        self.die_timer = self.create_timer(HEARTBEAT_TIMEOUT, self.die)
        self.stop()

    def drive(self, channel: int, effort: float) -> None:
        if self.dead:
            return
        count = counts(effort)
        nav.set_pwm_channel_value(channel, count)
        self.get_logger().debug(f"channel {channel}: {effort:.2f} -> {count}")

    def stop(self) -> None:
        for channel in (LEFT_CHANNEL, RIGHT_CHANNEL):
            nav.set_pwm_channel_value(channel, counts(0.0))

    def heartbeat(self, _msg: Empty) -> None:
        if self.dead:
            self.get_logger().info("heartbeat is back, thrusters live")
        self.dead = False
        self.die_timer.reset()

    def die(self) -> None:
        if not self.dead:
            self.get_logger().warning(
                f"no heartbeat for {HEARTBEAT_TIMEOUT}s, cutting thrust",
            )
        self.stop()
        self.dead = True  # after the thrusters are already at neutral


def main():
    nav.set_navigator_version(nav.NavigatorVersion.Version2)
    nav.init()
    nav.set_pwm_freq_hz(PWM_FREQ_HZ)
    nav.set_pwm_enable(True)

    rclpy.init()
    node = Thrusters()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Not node.stop(): the node may be half torn down by now.
        for channel in (LEFT_CHANNEL, RIGHT_CHANNEL):
            nav.set_pwm_channel_value(channel, counts(0.0))
        node.destroy_node()


if __name__ == "__main__":
    main()
