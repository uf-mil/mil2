from __future__ import annotations

import subprocess


def get_ros2_topics() -> list[str]:
    """
    Get ROS 2 topics from the active ROS graph.

    :raises RuntimeError: If ROS 2 CLI is found but topic listing fails.
    :raises FileNotFoundError: If ROS 2 CLI is not available.
    :return: Topic names from `ros2 topic list`.
    :rtype: list[str]
    """
    command = ("ros2", "topic", "list")
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError as e:
        raise FileNotFoundError(
            "No ROS 2 CLI found. Ensure 'ros2' is installed and in PATH.",
        ) from e

    if result.returncode != 0:
        raise RuntimeError(
            f"Failed to list ROS 2 topics with '{' '.join(command)}': {result.stderr.strip()}",
        )

    return [line.strip() for line in result.stdout.splitlines() if line.strip()]
