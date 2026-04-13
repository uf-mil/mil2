from __future__ import annotations

import os
import time
from contextlib import suppress

from .utils import canonical_topic_name


class _RosGraphQuery:
    """Own a temporary ROS 2 node used for graph inspection queries."""

    def __init__(self) -> None:
        self._rclpy = None
        self._context = None
        self._executor = None
        self._node = None
        self._owns_context = False

    def __enter__(self) -> _RosGraphQuery:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
        except ImportError as exc:
            raise RuntimeError(
                "rclpy is required to inspect the ROS 2 graph.",
            ) from exc

        self._rclpy = rclpy

        default_context = rclpy.get_default_context()
        if default_context.ok():
            self._context = default_context
        else:
            self._context = rclpy.context.Context()
            try:
                rclpy.init(context=self._context)
            except Exception as exc:
                raise RuntimeError(
                    "Failed to initialize ROS 2 rclpy context for graph queries.",
                ) from exc
            self._owns_context = True

        try:
            self._node = Node(
                f"robogym_graph_query_{os.getpid()}_{time.monotonic_ns()}",
                context=self._context,
            )
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
        except Exception as exc:
            self.__exit__(None, None, None)
            raise RuntimeError(
                "Failed to create ROS 2 graph query node.",
            ) from exc

        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        if self._executor is not None and self._node is not None:
            with suppress(Exception):
                self._executor.remove_node(self._node)
            self._executor.shutdown()
        self._executor = None

        if self._node is not None:
            self._node.destroy_node()
        self._node = None

        if (
            self._owns_context
            and self._context is not None
            and self._context.ok()
            and self._rclpy is not None
        ):
            self._rclpy.shutdown(context=self._context)
        self._context = None
        self._rclpy = None
        self._owns_context = False

    def snapshot(self) -> list[tuple[str, list[str]]]:
        if self._node is None:
            raise RuntimeError("ROS 2 graph query node is not active.")
        return [
            (topic_name, list(topic_types))
            for topic_name, topic_types in self._node.get_topic_names_and_types()
        ]

    def spin_once(self, timeout_s: float) -> None:
        if self._executor is not None:
            self._executor.spin_once(timeout_sec=timeout_s)


def _open_graph_query() -> _RosGraphQuery:
    return _RosGraphQuery()


def _validate_non_negative_timeout(timeout_s: float) -> None:
    if timeout_s < 0:
        raise ValueError("timeout_s must be non-negative.")


def get_topic_names_and_types(
    *,
    timeout_s: float = 0.0,
) -> list[tuple[str, list[str]]]:
    """
    Return the current ROS 2 graph topics and their advertised message types.

    When `timeout_s` is positive, the query waits up to that long for graph
    discovery to surface at least one topic.
    """
    _validate_non_negative_timeout(timeout_s)

    with _open_graph_query() as query:
        deadline = time.monotonic() + timeout_s
        while True:
            snapshot = query.snapshot()
            if snapshot or timeout_s == 0:
                return snapshot

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return snapshot

            query.spin_once(min(0.1, remaining))


def get_topic_names(
    *,
    timeout_s: float = 0.0,
) -> list[str]:
    """Return ROS 2 topic names from the active graph."""
    return [
        topic_name
        for topic_name, _topic_types in get_topic_names_and_types(timeout_s=timeout_s)
    ]


def resolve_topics_in_graph(
    topics: list[str],
    *,
    timeout_s: float = 2.0,
    operation: str,
) -> dict[str, tuple[str, list[str]]]:
    """
    Resolve requested topic names against the active ROS 2 graph.

    Returns a mapping from each original requested topic string to:
      - the graph-resolved topic name
      - the list of message types currently reported for that topic
    """
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive.")

    requested_topics = list(topics)
    if not requested_topics:
        return {}

    with _open_graph_query() as query:
        deadline = time.monotonic() + timeout_s
        resolved_topics: dict[str, tuple[str, list[str]]] = {}
        missing_topics = list(requested_topics)
        type_unresolved_topics: list[str] = []

        while True:
            snapshot = query.snapshot()
            topic_type_map = {
                topic_name: list(topic_types) for topic_name, topic_types in snapshot
            }
            available_lookup = {
                normalized_name: topic_name
                for topic_name in topic_type_map
                if (normalized_name := canonical_topic_name(topic_name))
            }

            resolved_topics = {}
            missing_topics = []
            type_unresolved_topics = []

            for topic in requested_topics:
                resolved_topic = available_lookup.get(canonical_topic_name(topic))
                if resolved_topic is None:
                    missing_topics.append(topic)
                    continue

                topic_types = [t for t in topic_type_map.get(resolved_topic, []) if t]
                if not topic_types:
                    type_unresolved_topics.append(topic)
                    continue

                resolved_topics[topic] = (resolved_topic, topic_types)

            if len(resolved_topics) == len(requested_topics):
                return resolved_topics

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            query.spin_once(min(0.1, remaining))

    if missing_topics:
        raise RuntimeError(
            f"Could not {operation} because these topics were not found "
            f"in the ROS 2 graph: {missing_topics}",
        )

    raise RuntimeError(
        "Failed to resolve ROS 2 topic types for: " f"{type_unresolved_topics}.",
    )
