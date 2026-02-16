from __future__ import annotations

import shlex
import subprocess
from pathlib import Path


def _extract_world_from_tokens(tokens: list[str]) -> str | None:
    """
    Return the first world-file argument found in a tokenized command line.

    The function strips surrounding quotes, removes a `file://` prefix when
    present, expands user-home markers, and accepts `.sdf` or `.world` paths.

    :param tokens: Tokenized process command line.
    :type tokens: list[str]
    :return: Normalized world file path if present, otherwise `None`.
    :rtype: str | None
    """
    for token in tokens:
        candidate = token.strip().strip("'").strip('"')
        if candidate.startswith("file://"):
            candidate = candidate.removeprefix("file://")

        if candidate.endswith((".sdf", ".world")):
            return str(Path(candidate).expanduser())

    return None


def get_gazebo_world_file() -> str:
    """
    Resolve the world file used by a currently running Gazebo simulation.

    Scans process arguments from `ps`, filters Gazebo-like commands, then
    parses each candidate command to locate a `.sdf` or `.world` argument.
    When multiple Gazebo processes are present, the highest PID is checked
    first as a heuristic for the most recent process.

    :raises RuntimeError: If Gazebo is running but no world file argument can be identified, or if process listing fails.
    :raises FileNotFoundError: If no Gazebo simulation process is found.
    :return: Expanded world file path from Gazebo process arguments.
    :rtype: str
    """
    result = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        capture_output=True,
        text=True,
        check=False,
    )

    if result.returncode != 0:
        raise RuntimeError(
            f"Failed to inspect running processes: {result.stderr.strip()}",
        )

    matches: list[tuple[int, str]] = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line:
            continue

        pid_str, _, command = line.partition(" ")
        if not pid_str.isdigit() or not command:
            continue

        if (
            "gz sim" in command
            or "ign gazebo" in command
            or " gazebo " in f" {command} "
        ):
            matches.append((int(pid_str), command))

    if not matches:
        raise FileNotFoundError("No running Gazebo simulation process was found.")

    for _, command in sorted(matches, key=lambda item: item[0], reverse=True):
        try:
            tokens = shlex.split(command)
        except ValueError:
            continue

        world_file = _extract_world_from_tokens(tokens)
        if world_file:
            return world_file

    raise RuntimeError(
        "Gazebo process found, but no '.sdf' or '.world' argument was detected.",
    )
