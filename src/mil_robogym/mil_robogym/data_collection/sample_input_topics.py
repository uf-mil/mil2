from __future__ import annotations

import subprocess

from .get_ros2_topics import get_ros2_topics
from .types import RoboGymProject


def _canonical_topic_name(topic: str) -> str:
    """Normalize a topic name by trimming whitespace and leading slashes."""
    return topic.strip().lstrip("/")


def _default_message_as_dict(msg_type: str) -> object:
    """
    Build a default-initialized ROS 2 message for `msg_type` and convert it
    into an ordered dictionary-compatible Python object.
    """
    try:
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        raise RuntimeError(
            "rosidl_runtime_py is required to derive values from ROS 2 message types.",
        ) from e

    try:
        msg_cls = get_message(msg_type)
        return message_to_ordereddict(msg_cls())
    except Exception as e:
        raise RuntimeError(
            f"Failed to resolve default ROS 2 message for type '{msg_type}'.",
        ) from e


def _flatten_value(value: object, prefix: str, out: dict[str, object]) -> None:
    """
    Flatten nested dictionaries/lists into dot and bracket notation keys.

    Examples:
        pose.position.x
        covariance[0]
    """
    if isinstance(value, dict):
        if not value and prefix:
            out[prefix] = {}
            return
        for key in sorted(value, key=str):
            key_str = str(key)
            child_prefix = f"{prefix}.{key_str}" if prefix else key_str
            _flatten_value(value[key], child_prefix, out)
        return

    if isinstance(value, list):
        if not value and prefix:
            out[prefix] = []
            return
        for index, item in enumerate(value):
            child_prefix = f"{prefix}[{index}]" if prefix else f"[{index}]"
            _flatten_value(item, child_prefix, out)
        return

    if prefix:
        out[prefix] = value
    else:
        out["value"] = value


def sample_input_topics(
    project: RoboGymProject,
    *,
    timeout_s: float = 2.0,
) -> dict[str, dict[str, object]]:
    """
    Resolve each project input topic to a ROS 2 message type and return
    flattened default values for that type.

    Uses:
        - `get_ros2_topics()` to verify each input topic is present
        - `ros2 topic type <topic>` to resolve the topic message type
        - ROS 2 message introspection to instantiate default message values

    Returns:
        A mapping from the original input topic string to flattened field/value
        pairs, where nested dictionaries use dot notation and lists use index
        notation.

    Raises:
        ValueError if `timeout_s` is not positive.
        RuntimeError if topic discovery, type resolution, or introspection fails.
    """
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive.")

    input_topics = list(project["input_topics"])
    if not input_topics:
        return {}

    try:
        available_topics = get_ros2_topics()
    except (RuntimeError, FileNotFoundError) as e:
        raise RuntimeError(
            "Failed to list ROS 2 topics before sampling input topics.",
        ) from e

    available_lookup = {
        _canonical_topic_name(topic): topic
        for topic in available_topics
        if _canonical_topic_name(topic)
    }

    missing_topics = [
        topic
        for topic in input_topics
        if _canonical_topic_name(topic) not in available_lookup
    ]
    if missing_topics:
        raise RuntimeError(
            "Could not sample input topics because these topics were not found "
            f"in the ROS 2 graph: {missing_topics}",
        )

    sampled_topics: dict[str, dict[str, object]] = {}
    for input_topic in input_topics:
        resolved_topic = available_lookup[_canonical_topic_name(input_topic)]
        command = ("ros2", "topic", "type", resolved_topic)
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=False,
                timeout=timeout_s,
            )
        except FileNotFoundError as e:
            raise RuntimeError(
                "No ROS 2 CLI found. Ensure 'ros2' is installed and in PATH.",
            ) from e
        except subprocess.TimeoutExpired as e:
            raise RuntimeError(
                f"Timed out after {timeout_s}s while resolving type for topic '{resolved_topic}'.",
            ) from e

        if result.returncode != 0:
            raise RuntimeError(
                "Failed to resolve topic type for "
                f"'{resolved_topic}' with '{' '.join(command)}': {result.stderr.strip()}",
            )

        if not result.stdout.strip():
            raise RuntimeError(
                f"Received empty topic type output for topic '{resolved_topic}'.",
            )

        msg_type = result.stdout.strip().splitlines()[0].strip()
        if not msg_type:
            raise RuntimeError(
                f"Resolved an invalid topic type for topic '{resolved_topic}'.",
            )

        parsed = _default_message_as_dict(msg_type)
        flattened: dict[str, object] = {}
        _flatten_value(parsed, "", flattened)
        sampled_topics[input_topic] = flattened

    return sampled_topics
