"""Unit tests for shared ROS graph query helpers."""

from __future__ import annotations

import pytest

from mil_robogym.data_collection.get_ros2_topics import get_ros2_topics
from mil_robogym.data_collection.ros_graph import (
    get_topic_names,
    resolve_topics_in_graph,
)
from mil_robogym.data_collection.sample_input_topics import (
    resolve_topic_message_types,
)


class _FakeQuery:
    def __init__(self, snapshots: list[list[tuple[str, list[str]]]]) -> None:
        self._snapshots = snapshots
        self._index = 0
        self.spin_calls: list[float] = []

    def __enter__(self) -> _FakeQuery:
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        return None

    def snapshot(self) -> list[tuple[str, list[str]]]:
        return [
            (topic_name, list(topic_types))
            for topic_name, topic_types in self._snapshots[self._index]
        ]

    def spin_once(self, timeout_s: float) -> None:
        self.spin_calls.append(timeout_s)
        if self._index < len(self._snapshots) - 1:
            self._index += 1


class _SteppingClock:
    def __init__(self, *, step_s: float) -> None:
        self._value = 0.0
        self._step_s = step_s

    def __call__(self) -> float:
        current = self._value
        self._value += self._step_s
        return current


def _install_fake_query(
    monkeypatch: pytest.MonkeyPatch,
    snapshots: list[list[tuple[str, list[str]]]],
) -> _FakeQuery:
    fake_query = _FakeQuery(snapshots)
    monkeypatch.setattr(
        "mil_robogym.data_collection.ros_graph._open_graph_query",
        lambda: fake_query,
    )
    return fake_query


def test_get_topic_names_returns_names_from_graph(monkeypatch: pytest.MonkeyPatch):
    _install_fake_query(
        monkeypatch,
        [
            [
                ("/camera/image_raw", ["sensor_msgs/msg/Image"]),
                ("/ping", ["std_msgs/msg/Float64"]),
            ],
        ],
    )

    assert get_topic_names() == ["/camera/image_raw", "/ping"]


def test_resolve_topics_in_graph_waits_for_requested_topic(
    monkeypatch: pytest.MonkeyPatch,
):
    fake_query = _install_fake_query(
        monkeypatch,
        [
            [("/clock", ["rosgraph_msgs/msg/Clock"])],
            [("/ping", ["std_msgs/msg/Float64"])],
        ],
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.ros_graph.time.monotonic",
        _SteppingClock(step_s=0.05),
    )

    resolved = resolve_topics_in_graph(
        ["ping"],
        timeout_s=0.2,
        operation="resolve topic types",
    )

    assert resolved == {"ping": ("/ping", ["std_msgs/msg/Float64"])}
    assert fake_query.spin_calls


def test_resolve_topics_in_graph_raises_when_topic_missing(
    monkeypatch: pytest.MonkeyPatch,
):
    _install_fake_query(
        monkeypatch,
        [[("/clock", ["rosgraph_msgs/msg/Clock"])]],
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.ros_graph.time.monotonic",
        _SteppingClock(step_s=0.2),
    )

    with pytest.raises(RuntimeError, match="these topics were not found"):
        resolve_topics_in_graph(
            ["/ping"],
            timeout_s=0.1,
            operation="collect topic payloads",
        )


def test_get_ros2_topics_uses_shared_graph_helper(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(
        "mil_robogym.data_collection.get_ros2_topics.get_topic_names",
        lambda: ["/foo", "/bar"],
    )

    assert get_ros2_topics() == ["/foo", "/bar"]


def test_resolve_topic_message_types_uses_first_reported_type(
    monkeypatch: pytest.MonkeyPatch,
):
    seen: list[tuple[list[str], float, str]] = []

    def _fake_resolve(
        topics: list[str],
        *,
        timeout_s: float,
        operation: str,
    ) -> dict[str, tuple[str, list[str]]]:
        seen.append((list(topics), timeout_s, operation))
        return {
            "/ping": (
                "/ping",
                ["std_msgs/msg/Float64", "example_msgs/msg/Fallback"],
            ),
        }

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.resolve_topics_in_graph",
        _fake_resolve,
    )

    resolved = resolve_topic_message_types(["/ping"], timeout_s=0.75)

    assert resolved == {"/ping": "std_msgs/msg/Float64"}
    assert seen == [(["/ping"], 0.75, "resolve topic types")]
