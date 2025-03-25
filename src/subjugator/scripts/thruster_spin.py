#!/usr/bin/env python3

"""
Used for spinning the thrusters of sub9!
"""

import argparse

import rclpy
from rclpy.node import Node
from subjugator_msgs.msg import ThrusterEfforts


class ThrusterSpinner(Node):
    def __init__(self, thrusters: list[str], rate: float):
        super().__init__("thruster_spinner")
        self.pub = self.create_publisher(ThrusterEfforts, "/thruster_efforts", 10)
        self.rate = rate
        self.timer = self.create_timer(0.5, self.spin)
        self.thrusters = thrusters
        print(f"Spinning thrusters: {self.thrusters} at rate: {self.rate}")

    # 'FLH' --> 'thruster_flh'
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
    parser.add_argument(
        "--zero",
        action="store_true",
        help="Set zero rate (0), useful for testing thruster pipeline with no effort",
    )
    parser.add_argument("--slow", action="store_true", help="Set slow rate (0.2)")
    parser.add_argument("--medium", action="store_true", help="Set medium rate (0.5)")
    parser.add_argument("--fast", action="store_true", help="Set fast rate (1)")
    parser.add_argument(
        "thrusters",
        nargs="*",
        help=f"List of thruster names to spin ({options})",
    )

    args = parser.parse_args()

    # Define default rates based on arguments
    if args.zero:
        rate = 0.0
    elif args.slow:
        rate = 0.2
    elif args.medium:
        rate = 0.5
    elif args.fast:
        rate = 0.9
    elif args.rate > 0:
        rate = args.rate
    else:
        raise ValueError(
            "No rate specified! Use --rate, --zero, --slow, --medium, or --fast.",
        )

    # Determine thrusters to spin
    thrusters = []
    if args.all:
        thrusters = options
    elif args.thrusters:
        thrusters = args.thrusters
    else:
        raise ValueError(
            f"No thrusters specified! Use --all or list specific thrusters ({options}).",
        )

    # Create node
    rclpy.init()
    node = ThrusterSpinner(thrusters, rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping...")
        node.reset()
    finally:
        # node.destroy_node()
        # rclpy.shutdown()
        pass


if __name__ == "__main__":
    main()
