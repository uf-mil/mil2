#!/usr/bin/env python3
"""Publish synthetic tracked detections for the persistent obstacle map.

The historical executable name is retained, but this publishes MarkerArray,
not OccupancyGrid. Repeated detections satisfy ObstacleMap's confirmation gate.
Set publish_odometry:=true for a stationary simulated boat at the map origin.
The obstacle at (10, 0) blocks the direct route to the test goal at (20, 0).
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray


def main():
    rclpy.init()
    node = Node("prop_planner_mock_map")
    topic = node.declare_parameter("tracks_topic", "tracked_markers").value
    frame = node.declare_parameter("map_frame", "map").value
    publisher = node.create_publisher(MarkerArray, topic, 10)

    publish_odometry = node.declare_parameter("publish_odometry", False).value
    odom_topic = node.declare_parameter("odom_topic", "/odometry/filtered/global").value
    odom_publisher = (
        node.create_publisher(Odometry, odom_topic, qos_profile_sensor_data)
        if publish_odometry
        else None
    )

    def publish():
        if odom_publisher is not None:
            odom = Odometry()
            odom.header.frame_id = frame
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom_publisher.publish(odom)
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
    if publish_odometry:
        node.get_logger().info(
            f"Publishing synthetic stationary odometry on {odom_topic}",
        )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
