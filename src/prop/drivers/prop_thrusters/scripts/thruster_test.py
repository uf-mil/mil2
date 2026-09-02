#!/usr/bin/env python3
"""
Spin a thruster at a fixed effort for a fixed time, for bench testing.

    ros2 run prop_thrusters thruster_test.py -f 0.3 -d 5 -m left

Publishes the same topics the controller does, so thrusters.py has to be
running. The heartbeat is sent too, otherwise the driver would cut thrust two
seconds in.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty, Float32

HEARTBEAT_HZ = 5.0


class ThrusterTest(Node):
    def __init__(self, motor: str):
        super().__init__("thruster_test")

        self.pubs = []
        if motor in ("left", "both"):
            self.pubs.append(self.create_publisher(Float32, "thrusters/left", 10))
        if motor in ("right", "both"):
            self.pubs.append(self.create_publisher(Float32, "thrusters/right", 10))

        self.beat_pub = self.create_publisher(Empty, "thrusters/heartbeat", 10)
        self.create_timer(1 / HEARTBEAT_HZ, lambda: self.beat_pub.publish(Empty()))

    def publish(self, effort: float) -> None:
        for pub in self.pubs:
            pub.publish(Float32(data=float(effort)))

    def hold(self, effort: float, seconds: float) -> None:
        """Hold an effort, spinning so the heartbeat keeps going out."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.publish(effort)
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop(self) -> None:
        self.publish(0.0)

    def wait_for_driver(self, timeout: float = 3.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if all(pub.get_subscription_count() > 0 for pub in self.pubs):
                return True
            time.sleep(0.1)

        topics = " and ".join(pub.topic_name for pub in self.pubs)
        self.get_logger().error(
            f"nothing subscribed to {topics} after {timeout}s - "
            "is thrusters.py running?",
        )
        return False


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Spin the thrusters at a set effort for a set amount of time.",
    )
    parser.add_argument(
        "-f",
        "--force",
        type=float,
        required=True,
        help="commanded effort, -1.0 (full reverse) to 1.0 (full forward)",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        required=True,
        help="seconds to spin",
    )
    parser.add_argument(
        "-m",
        "--motor",
        choices=("left", "right", "both"),
        default="both",
        help="which thruster(s) to spin (default: both)",
    )
    opts = parser.parse_args()

    if not -1.0 <= opts.force <= 1.0:
        parser.error("--force must be between -1.0 and 1.0")
    if opts.duration <= 0.0:
        parser.error("--duration must be greater than 0")

    # rclpy's own SIGINT handler invalidates the context before we can send a
    # stop, which would leave the thrusters spinning. Let Python raise
    # KeyboardInterrupt instead so the finally below still has a live context.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = ThrusterTest(opts.motor)
    try:
        if not node.wait_for_driver():
            return

        node.get_logger().info("arming for 1s (neutral)")
        node.hold(0.0, 1.0)

        node.get_logger().info(f"spinning at {opts.force} for {opts.duration}s")
        node.hold(opts.force, opts.duration)
    except KeyboardInterrupt:
        node.get_logger().warning("interrupted")
    finally:
        node.stop()
        node.get_logger().info("stopped")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
