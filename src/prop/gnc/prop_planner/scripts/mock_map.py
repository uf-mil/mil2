#!/usr/bin/env python3
"""Publish synthetic tracked detections for the persistent obstacle map.

The historical executable name is retained, but this publishes MarkerArray,
not OccupancyGrid. Repeated detections satisfy ObstacleMap's confirmation gate.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def main():
    rclpy.init()
    node = Node("prop_planner_mock_map")
    topic = node.declare_parameter("tracks_topic", "tracked_markers").value
    frame = node.declare_parameter("map_frame", "map").value
    publisher = node.create_publisher(MarkerArray, topic, 10)

    def publish():
        clear = Marker()
        clear.action = Marker.DELETEALL
        obstacle = Marker()
        obstacle.header.frame_id = frame
        obstacle.header.stamp = node.get_clock().now().to_msg()
        obstacle.ns = "mock_tracks"
        obstacle.id = 1
        obstacle.type = Marker.CUBE
        obstacle.action = Marker.ADD
        obstacle.pose.position.x = 10.0
        obstacle.pose.orientation.w = 1.0
        obstacle.scale.x = 2.0
        obstacle.scale.y = 8.0
        obstacle.scale.z = 1.0
        obstacle.color.r = 1.0
        obstacle.color.a = 1.0
        publisher.publish(MarkerArray(markers=[clear, obstacle]))

    node.create_timer(0.2, publish)
    node.get_logger().info(f"Publishing synthetic tracked detections on {topic}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
