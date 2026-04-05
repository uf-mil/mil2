from __future__ import annotations

from .ros_graph import get_topic_names


def get_ros2_topics() -> list[str]:
    """
    Get all topic names from the active ROS 2 graph.

    Uses a temporary `rclpy` node to query the graph and returns topic names.
    """
    return get_topic_names()
