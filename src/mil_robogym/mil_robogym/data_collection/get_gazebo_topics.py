from __future__ import annotations

import subprocess


def get_gazebo_topics() -> list[str]:
    """
    Get Gazebo transport topics from a running simulation.

    :raises RuntimeError: If Gazebo CLI is found but topic listing fails.
    :raises FileNotFoundError: If no supported Gazebo CLI is available.
    :return: Topic names from Gazebo transport.
    :rtype: list[str]
    """
    commands = (
        ("gz", "topic", "-l"),
        ("ign", "topic", "-l"),
    )

    for command in commands:
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=False,
            )
        except FileNotFoundError:
            continue

        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to list Gazebo topics with '{' '.join(command)}': {result.stderr.strip()}",
            )

        return [line.strip() for line in result.stdout.splitlines() if line.strip()]

    raise FileNotFoundError(
        "No Gazebo CLI found. Install Gazebo and ensure either 'gz' or 'ign' is in PATH.",
    )
