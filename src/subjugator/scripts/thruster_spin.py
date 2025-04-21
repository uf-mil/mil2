#!/usr/bin/env python3

"""
Used for spinning the thrusters of sub9!
"""

import argparse

import rclpy
from rclpy.node import Node
from subjugator_msgs.msg import ThrusterEfforts


class ThrusterSpinner(Node):
    def __init__(self, thrusters: list[str], rate: float, timeout: float | None = None):
        super().__init__("thruster_spinner")
        self.pub = self.create_publisher(ThrusterEfforts, "/thruster_efforts", 10)
        self.rate = rate
        self.thrusters = thrusters
        self.spin_timer = self.create_timer(0.5, self.spin)
        self.shutdown_timer = None

        if timeout is not None:
            self.shutdown_timer = self.create_timer(timeout, self.shutdown)

        print(
            f"Spinning thrusters: {self.thrusters} at rate: {self.rate} for {timeout if timeout else '∞'} seconds",
        )

    def _option_to_field_name(self, name: str) -> str:
        return f"thrust_{name.lower()}"

    def spin(self):
        effort = ThrusterEfforts()
        for thruster in self.thrusters:
            setattr(effort, self._option_to_field_name(thruster), self.rate)
        self.pub.publish(effort)

    def reset(self):
        effort = ThrusterEfforts()
        for thruster in self.thrusters:
            setattr(effort, self._option_to_field_name(thruster), 0.0)
        self.pub.publish(effort)

    def shutdown(self):
        print("Timeout reached. Stopping thrusters.")
        self.reset()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="Spin thrusters for the autonomous boat",
    )
    options = ["FLH", "FLV", "FRH", "FRV", "BLH", "BLV", "BRH", "BRV"]

    parser.add_argument("--all", action="store_true", help="Spin all thrusters")
    parser.add_argument(
        "--rate",
        type=float,
        default=0,
        help="Set thruster spin rate (-1, 1)",
    )
    parser.add_argument("--zero", action="store_true", help="Set zero rate (0)")
    parser.add_argument("--slow", action="store_true", help="Set slow rate (0.2)")
    parser.add_argument("--medium", action="store_true", help="Set medium rate (0.5)")
    parser.add_argument("--fast", action="store_true", help="Set fast rate (1)")
    parser.add_argument(
        "--timeout",
        type=float,
        help="Spin duration in seconds (then auto-stop)",
    )
    parser.add_argument(
        "thrusters",
        nargs="*",
        help=f"List of thruster names to spin ({options})",
    )

    args = parser.parse_args()

    if args.zero:
        rate = 0.0
    elif args.slow:
        rate = 0.2
    elif args.medium:
        rate = 0.5
    elif args.fast:
        rate = 0.9
    elif -1 < args.rate < 1:
        rate = args.rate
    else:
        raise ValueError("No valid rate specified.")

    thrusters = options if args.all else args.thrusters
    if not thrusters:
        raise ValueError(
            f"No thrusters specified! Use --all or list specific thrusters ({options}).",
        )

    rclpy.init()
    node = ThrusterSpinner(thrusters, rate, timeout=args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt. Stopping...")
        node.reset()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
