#!/usr/bin/env python3
"""
Enable localization, the controller and the planner once the simulation is up.

    ros2 run prop_gazebo sim_autostart.py

What prop-start-localization, start-controller and start-planning do by hand,
in that order, with a pause after localization so the controller's first
station-keeping goal is an estimate that has settled rather than a datum still
on the move. Exits when done.
"""

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool


def call(node: Node, service_type, name: str, request) -> None:
    client = node.create_client(service_type, name)
    while not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info(f"waiting for {name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info(f"called {name}")


def main():
    rclpy.init()
    node = Node("sim_autostart")
    settle = node.declare_parameter("settle", 5.0).value

    call(node, Empty, "/ekf_local/enable", Empty.Request())
    call(node, Empty, "/ekf_global/enable", Empty.Request())
    node.get_logger().info(f"localization on, {settle:.0f} s for it to settle")
    time.sleep(settle)

    call(node, SetBool, "/pid_controller/enable", SetBool.Request(data=True))
    call(node, SetBool, "/planner/enable", SetBool.Request(data=True))
    node.get_logger().info("holding station; send a goal on goal_pose")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
