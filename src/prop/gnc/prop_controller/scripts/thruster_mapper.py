#!/usr/bin/env python3
"""
Turn the controller's wrench into an effort for each thruster.

    in   cmd_wrench  geometry_msgs/Wrench in the body frame: force.x is surge,
                     force.y sway, torque.z yaw. From pid_controller.
    out  thrusters/left, thrusters/right, thrusters/bow  std_msgs/Float32, -1.0 to 1.0
         thrusters/heartbeat                             std_msgs/Empty

Two thrusters push along x either side of the centreline; the bow thruster
pushes along y ahead of the centre of mass. Three thrusters for three axes is
an exact solve: the bow thruster takes the sway, the pair's difference takes
the yaw less what the bow thruster's lever adds, and the pair's sum takes the
surge. When that asks more of a thruster than it has, every effort is scaled
down by the same factor, so the boat gets less of the wrench but never a
different one. Letting one axis win instead would have the asymmetric aft
thrusters (55 N ahead, 28 N astern) shove the boat forward whenever it sways
or turns hard.

The heartbeat goes out with every command, so the driver's failsafe trips
exactly when the controller stops talking.
"""

import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from std_msgs.msg import Empty, Float32


class ThrusterMapper(Node):
    def __init__(self):
        super().__init__("thruster_mapper")
        self.thruster_y = self.declare_parameter("thruster_y", 0.27305).value
        self.bow_x = self.declare_parameter("bow_x", 0.4).value
        aft = (
            self.declare_parameter("max_force_pos", 55.21).value,
            self.declare_parameter("max_force_neg", 27.56).value,
        )
        bow = (
            self.declare_parameter("bow_force_pos", 51.5).value,
            self.declare_parameter("bow_force_neg", 40.2).value,
        )
        self.limits = {"left": aft, "right": aft, "bow": bow}

        self.pubs = {
            name: self.create_publisher(Float32, f"thrusters/{name}", 10)
            for name in self.limits
        }
        self.beat_pub = self.create_publisher(Empty, "thrusters/heartbeat", 10)
        self.create_subscription(Wrench, "cmd_wrench", self.on_wrench, 10)

    def on_wrench(self, msg: Wrench) -> None:
        bow = msg.force.y
        differential = (msg.torque.z - self.bow_x * bow) / self.thruster_y / 2.0
        forces = {
            "left": msg.force.x / 2.0 - differential,
            "right": msg.force.x / 2.0 + differential,
            "bow": bow,
        }

        # Effort is the fraction of what a thruster can give in that direction,
        # so scaling efforts scales forces by the same factor and the wrench
        # keeps its direction.
        efforts = {name: self.effort(name, force) for name, force in forces.items()}
        peak = max(1.0, *(abs(e) for e in efforts.values()))
        for name, effort in efforts.items():
            self.pubs[name].publish(Float32(data=effort / peak))
        self.beat_pub.publish(Empty())

    def effort(self, name: str, force: float) -> float:
        pos, neg = self.limits[name]
        return force / pos if force >= 0.0 else force / neg


def main():
    rclpy.init()
    node = ThrusterMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
