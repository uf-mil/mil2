#!/usr/bin/env python3
"""Publish a retained test grid with a wall and a gap; no real map input.

OccupancyGrid is the planner's temporary map interface. If the real mapping
system uses another data structure, a future adapter should convert that map
into the grid representation expected by the planner.
"""
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    rclpy.init()
    node = Node("prop_planner_mock_map")
    topic = node.declare_parameter("map_topic", "/prop_planner/mock_map").value
    publisher = node.create_publisher(
        OccupancyGrid, topic,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.header.stamp = node.get_clock().now().to_msg()
    grid.info.resolution = 1.0
    grid.info.width = 20
    grid.info.height = 20
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 400
    for y in range(15):
        grid.data[y * 20 + 10] = 100
    publisher.publish(grid)
    node.get_logger().info(f"Published synthetic 20 x 20 grid on {topic}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
