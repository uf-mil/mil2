from __future__ import annotations

import os
import time
from typing import Any

from .ros_graph import resolve_topics_in_graph
from .types import RoboGymProjectYaml, SampledTopics
from .utils import flatten_value


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


def _dedupe_preserve_order(values: list[str]) -> list[str]:
    deduped: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value in seen:
            continue
        seen.add(value)
        deduped.append(value)
    return deduped


def _resolve_topics_in_graph(
    topics: list[str],
    *,
    timeout_s: float,
    operation: str,
) -> list[str]:
    resolved_topics = resolve_topics_in_graph(
        topics,
        timeout_s=timeout_s,
        operation=operation,
    )
    return [resolved_topic for resolved_topic, _topic_types in resolved_topics.values()]


def resolve_topic_message_types(
    topics: list[str],
    *,
    timeout_s: float = 2.0,
) -> dict[str, str]:
    """
    Resolve each topic to its ROS 2 message type.

    Returns:
        Mapping from the original topic string to the resolved message type.
    """
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive.")

    selected_topics = list(topics)
    if not selected_topics:
        return {}

    resolved_topics = resolve_topics_in_graph(
        selected_topics,
        timeout_s=timeout_s,
        operation="resolve topic types",
    )
    return {topic: resolved_topics[topic][1][0] for topic in selected_topics}


def _collect_topic_messages_once_via_subscriptions(
    topics: list[str],
    *,
    timeout_s: float,
) -> list[Any]:
    """
    Collect one live message per topic via temporary ROS 2 subscriptions.
    """
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        raise RuntimeError(
            "rclpy and rosidl_runtime_py are required to collect live topic messages via subscription.",
        ) from e

    ordered_topics = list(topics)
    unique_topics = _dedupe_preserve_order(ordered_topics)

    context = rclpy.context.Context()
    try:
        rclpy.init(context=context)
    except Exception as e:
        raise RuntimeError(
            "Failed to initialize ROS 2 rclpy context for topic sampling.",
        ) from e

    node: Node | None = None
    executor: SingleThreadedExecutor | None = None
    subscriptions: list[object] = []
    try:
        node = Node(
            f"robogym_topic_sampler_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        type_lookup: dict[str, str] = {}
        type_deadline = time.monotonic() + timeout_s
        while time.monotonic() < type_deadline:
            topics_and_types = dict(node.get_topic_names_and_types())
            for topic in unique_topics:
                if topic in type_lookup:
                    continue
                topic_types = topics_and_types.get(topic)
                if topic_types:
                    type_lookup[topic] = topic_types[0]

            if len(type_lookup) == len(unique_topics):
                break

            remaining = type_deadline - time.monotonic()
            if remaining <= 0:
                break
            executor.spin_once(timeout_sec=min(0.1, remaining))

        unresolved = [topic for topic in unique_topics if topic not in type_lookup]
        if unresolved:
            raise RuntimeError(
                "Failed to resolve ROS 2 topic types for: " f"{unresolved}.",
            )

        received_messages: dict[str, Any] = {}

        for topic in unique_topics:
            msg_type = type_lookup[topic]
            try:
                msg_cls = get_message(msg_type)
            except Exception as e:
                raise RuntimeError(
                    f"Failed to resolve ROS 2 message class for topic '{topic}' "
                    f"with type '{msg_type}'.",
                ) from e

            def _make_callback(topic_name: str):
                def _callback(msg: object) -> None:
                    if topic_name in received_messages:
                        return
                    received_messages[topic_name] = message_to_ordereddict(msg)

                return _callback

            subscriptions.append(
                node.create_subscription(
                    msg_cls,
                    topic,
                    _make_callback(topic),
                    qos_profile_sensor_data,
                ),
            )

        receive_deadline = time.monotonic() + timeout_s
        while len(received_messages) < len(unique_topics):
            remaining = receive_deadline - time.monotonic()
            if remaining <= 0:
                break
            executor.spin_once(timeout_sec=min(0.1, remaining))

        missing_topics = [
            topic for topic in unique_topics if topic not in received_messages
        ]
        if missing_topics:
            raise RuntimeError(
                f"Timed out after {timeout_s}s while collecting topic messages for: {missing_topics}",
            )

        return [received_messages[topic] for topic in ordered_topics]
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def sample_topics(
    topics: list[str],
    *,
    timeout_s: float = 2.0,
) -> SampledTopics:
    """
    Resolve each topic to a ROS 2 message type and return flattened defaults.

    Uses:
        - `rclpy` graph introspection to verify each topic is present
        - `Node.get_topic_names_and_types()` to resolve each topic message type
        - ROS 2 message introspection to instantiate default message values

    Returns:
        A mapping from the original topic string to flattened field/value
        pairs, where nested dictionaries use dot notation and lists use index
        notation.

    Raises:
        ValueError if `timeout_s` is not positive.
        RuntimeError if topic discovery, type resolution, or introspection fails.
    """
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive.")

    selected_topics = list(topics)
    if not selected_topics:
        return {}
    topic_message_types = resolve_topic_message_types(
        selected_topics,
        timeout_s=timeout_s,
    )

    sampled_topics: SampledTopics = {}
    for topic in selected_topics:
        msg_type = topic_message_types[topic]
        parsed = _default_message_as_dict(msg_type)
        flattened: dict[str, object] = {}
        flatten_value(parsed, "", flattened)
        sampled_topics[topic] = flattened

    return sampled_topics


def collect_topic_payloads_once(
    topics: list[str],
    *,
    timeout_s: float = 2.0,
) -> list[Any]:
    """
    Collect one live ROS 2 message from each topic using subscriptions.

    Returns:
      A list of parsed message payloads in the same order as `topics`.
    """
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive.")

    selected_topics = list(topics)
    if not selected_topics:
        return []
    resolved_topics = _resolve_topics_in_graph(
        selected_topics,
        timeout_s=timeout_s,
        operation="collect topic payloads",
    )
    return _collect_topic_messages_once_via_subscriptions(
        resolved_topics,
        timeout_s=timeout_s,
    )


def numerical_headers_from_topic_subtopics(
    topic_subtopics: dict[str, list[str]],
) -> list[str]:
    """
    Build stable CSV headers in `<topic>:<field>` order.
    """
    headers: list[str] = []
    for topic, fields in topic_subtopics.items():
        for field in fields:
            headers.append(f"{topic}:{field}")
    return headers


def collect_numeric_values_from_topic_subtopics(
    topic_subtopics: dict[str, list[str]],
    *,
    timeout_s: float = 2.0,
) -> list[bool | int | float | None]:
    """
    Collect numeric values for selected topic subtopics in stable header order.

    For missing or non-numeric fields, returns `None` in that position.
    """
    selected_topics = list(topic_subtopics)
    raw_messages = collect_topic_payloads_once(selected_topics, timeout_s=timeout_s)

    values: list[bool | int | float | None] = []
    for topic, message in zip(selected_topics, raw_messages):
        flattened: dict[str, object] = {}
        flatten_value(message, "", flattened)
        for field in topic_subtopics[topic]:
            value = flattened.get(field)
            if isinstance(value, (bool, int, float)):
                values.append(value)
            else:
                values.append(None)
    return values


def sample_project_topics(
    project: RoboGymProjectYaml,
    *,
    timeout_s: float = 2.0,
) -> tuple[SampledTopics, SampledTopics]:
    """
    Resolve input and output topic schemas for a project in one ROS 2 query pass.

    Returns:
      Tuple of (sampled_inputs, sampled_outputs)
    """
    input_topics = list(project["input_topics"])
    output_topics = list(project["output_topics"])
    selected_topics = _dedupe_preserve_order([*input_topics, *output_topics])
    sampled_all = sample_topics(selected_topics, timeout_s=timeout_s)

    sampled_inputs = {
        topic: {
            subfield: value
            for subfield, value in sampled_all[topic].items()
            if subfield in project["input_topics"][topic]
        }
        for topic in input_topics
    }

    sampled_outputs = {
        topic: {
            subfield: value
            for subfield, value in sampled_all[topic].items()
            if subfield in project["output_topics"][topic]
        }
        for topic in output_topics
    }

    return sampled_inputs, sampled_outputs


def sample_input_topics(
    project: RoboGymProjectYaml,
    *,
    timeout_s: float = 2.0,
) -> SampledTopics:
    """
    Resolve each project input topic to a ROS 2 message type and return
    flattened default values for that type.

    This is a project-shaped wrapper around `sample_project_topics`.

    Uses:
        - `sample_project_topics(project, timeout_s=...)`

    Returns:
        A mapping from the original input topic string to flattened field/value
        pairs, where nested dictionaries use dot notation and lists use index
        notation.

    Raises:
        ValueError if `timeout_s` is not positive.
        RuntimeError if topic discovery, type resolution, or introspection fails.
    """
    sampled_inputs, _sampled_outputs = sample_project_topics(
        project,
        timeout_s=timeout_s,
    )

    return sampled_inputs
