#!/usr/bin/env python3
"""
Stands in for the thruster driver on the boat.

    in   thrusters/left, thrusters/right  std_msgs/Float32, -1.0 to 1.0
         thrusters/heartbeat              std_msgs/Empty
    out  sim/thrust_left, sim/thrust_right  std_msgs/Float64, newtons

The rest of the stack talks normalized effort and Gazebo's PropThrusters
plugin wants newtons, which the bridge cannot convert between on its own.
Carries the same heartbeat failsafe as prop_thrusters, so a controller that
stops talking is caught in simulation as well as on the water.
"""

from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Float64

SIDES = ("left", "right")


class SimThrusters(Node):
    def __init__(self):
        super().__init__("sim_thrusters")

        self.max_pos = self.declare_parameter("max_force_pos", 55.21).value
        self.max_neg = self.declare_parameter("max_force_neg", 27.56).value
        timeout = self.declare_parameter("heartbeat_timeout", 2.0).value

        self.dead = False
        self.pubs = {
            side: self.create_publisher(Float64, f"sim/thrust_{side}", 10)
            for side in SIDES
        }
        for side in SIDES:
            self.create_subscription(
                Float32,
                f"thrusters/{side}",
                partial(self.drive, side),
                10,
            )
        self.create_subscription(Empty, "thrusters/heartbeat", self.heartbeat, 10)

        self.die_timer = self.create_timer(timeout, self.die)
        self.stop()

    def drive(self, side: str, msg: Float32) -> None:
        if self.dead:
            return
        effort = max(-1.0, min(1.0, msg.data))
        # Full effort astern is worth much less thrust than full effort ahead,
        # which is what the controller solved against.
        limit = self.max_pos if effort >= 0.0 else self.max_neg
        self.pubs[side].publish(Float64(data=effort * limit))

    def stop(self) -> None:
        for pub in self.pubs.values():
            pub.publish(Float64(data=0.0))

    def heartbeat(self, _msg: Empty) -> None:
        if self.dead:
            self.get_logger().info("heartbeat is back, thrusters live")
        self.dead = False
        self.die_timer.reset()

    def die(self) -> None:
        if not self.dead:
            self.get_logger().warning("no heartbeat, cutting thrust")
        self.stop()
        self.dead = True


def main():
    rclpy.init()
    node = SimThrusters()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
