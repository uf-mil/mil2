from __future__ import annotations

import queue
import threading
import time
from collections import defaultdict
from typing import Any, Callable

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from mil_robogym.clients.world_control_client import WorldControlClient


def _make_probe_qos() -> QoSProfile:
    qos = QoSProfile(depth=50)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    return qos


def _compute_hz_from_times(times: list[float]) -> float | None:
    if len(times) < 2:
        return None

    intervals = []
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt > 0:
            intervals.append(dt)

    if not intervals:
        return None

    avg_period = sum(intervals) / len(intervals)
    if avg_period <= 0:
        return None

    return 1.0 / avg_period


def _cleanup_ros(
    node: Any,
    executor: Any,
    subscriptions: list[Any],
    context: Any,
) -> None:
    def _ignore_errors(operation: Callable[[], None]) -> None:
        try:
            operation()
        except Exception:
            return

    if node is not None:
        for subscription in subscriptions:
            _ignore_errors(lambda: node.destroy_subscription(subscription))

    if executor is not None and node is not None:
        _ignore_errors(lambda: executor.remove_node(node))

    if node is not None:
        _ignore_errors(node.destroy_node)

    _ignore_errors(lambda: rclpy.shutdown(context=context))


def get_topic_hz(topic: str, timeout_s: float = 5.0) -> float | None:
    context = rclpy.context.Context()
    node = None
    executor = None
    subscriptions: list[Any] = []
    message_times: list[float] = []

    try:
        rclpy.init(args=None, context=context)

        node = rclpy.create_node(
            f"topic_hz_probe_{abs(hash(topic)) % 10_000_000}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        topic_types = dict(node.get_topic_names_and_types())
        ros_type_names = topic_types.get(topic)
        if not ros_type_names:
            return None

        msg_type = get_message(ros_type_names[0])

        def on_message(_msg: Any) -> None:
            message_times.append(time.monotonic())

        subscription = node.create_subscription(
            msg_type,
            topic,
            on_message,
            _make_probe_qos(),
        )
        subscriptions.append(subscription)

        end_time = time.monotonic() + timeout_s
        while time.monotonic() < end_time:
            executor.spin_once(timeout_sec=min(0.1, end_time - time.monotonic()))

        return _compute_hz_from_times(message_times)

    except Exception as exc:
        raise RuntimeError(f"Failed to measure topic hertz for '{topic}'.") from exc

    finally:
        _cleanup_ros(node, executor, subscriptions, context)


def start_topic_hz_lookup(
    topics: list[str],
    timeout_s: float = 5.0,
) -> tuple[queue.Queue[tuple[str, float | None]], threading.Event]:
    results: queue.Queue[tuple[str, float | None]] = queue.Queue()
    done = threading.Event()

    def worker() -> None:
        client = WorldControlClient()
        client.acquire_simulation_hold()
        valid_topics = [topic for topic in topics if topic != "No topics found"]

        context = rclpy.context.Context()
        node = None
        executor = None
        subscriptions: list[Any] = []
        message_times_by_topic: dict[str, list[float]] = defaultdict(list)
        failed_topics: set[str] = set()

        try:
            if not valid_topics:
                return

            rclpy.init(args=None, context=context)
            node = rclpy.create_node("topic_hz_lookup", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)

            topic_types = dict(node.get_topic_names_and_types())

            for topic in valid_topics:
                ros_type_names = topic_types.get(topic)
                if not ros_type_names:
                    results.put((topic, None))
                    failed_topics.add(topic)
                    continue

                try:
                    msg_type = get_message(ros_type_names[0])

                    def callback(_msg: Any, topic_name: str = topic) -> None:
                        message_times_by_topic[topic_name].append(time.monotonic())

                    subscription = node.create_subscription(
                        msg_type,
                        topic,
                        callback,
                        _make_probe_qos(),
                    )
                    subscriptions.append(subscription)
                except Exception:
                    results.put((topic, None))
                    failed_topics.add(topic)

            end_time = time.monotonic() + timeout_s
            while time.monotonic() < end_time:
                executor.spin_once(timeout_sec=min(0.1, end_time - time.monotonic()))

            for topic in valid_topics:
                if topic in failed_topics:
                    continue
                hz = _compute_hz_from_times(message_times_by_topic[topic])
                results.put((topic, hz))

        finally:
            _cleanup_ros(node, executor, subscriptions, context)
            client.release_simulation_hold()
            done.set()

    threading.Thread(target=worker, daemon=True).start()
    return results, done
