from __future__ import annotations

import subprocess


def get_ros2_topics() -> list[str]:
    """
    Get all topic names from the active ROS 2 graph.

    Uses `ros2 topic list` and returns non-empty lines as topic names.
    Raises RuntimeError when the command exits unsuccessfully.
    Raises FileNotFoundError when the ROS 2 CLI is unavailable.
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
