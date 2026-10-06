from __future__ import annotations

import pytest

from mil_robogym.data_collection.ros_graph import TopicGraphResolution
from mil_robogym.data_collection.topic_readiness import (
    TopicReadinessError,
    TopicReadinessReport,
    check_topics_ready,
    ensure_project_input_topics_ready,
)


def test_check_topics_ready_reports_success(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        "mil_robogym.data_collection.topic_readiness.query_topics_in_graph",
        lambda topics, *, timeout_s: TopicGraphResolution(
            requested_topics=list(topics),
            resolved_topics={
                "/imu/data": ("/imu/data", ["sensor_msgs/msg/Imu"]),
                "/dvl/data": ("/dvl/data", ["geometry_msgs/msg/TwistStamped"]),
            },
            missing_topics=[],
            type_unresolved_topics=[],
        ),
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.topic_readiness.collect_resolved_topic_payloads_once",
        lambda topics, *, timeout_s, topic_message_types: (
            {
                "/imu/data": {"orientation": {"x": 0.0}},
                "/dvl/data": {"twist": {"linear": {"x": 0.0}}},
            },
            [],
        ),
    )

    report = check_topics_ready(["/imu/data", "/dvl/data"], timeout_s=0.5)

    assert report.ready is True
    assert report.resolved_topics == {
        "/imu/data": "/imu/data",
        "/dvl/data": "/dvl/data",
    }
    assert report.missing_topics == []
    assert report.type_unresolved_topics == []
    assert report.silent_topics == []


def test_check_topics_ready_reports_missing_type_and_silent_topics(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "mil_robogym.data_collection.topic_readiness.query_topics_in_graph",
        lambda topics, *, timeout_s: TopicGraphResolution(
            requested_topics=list(topics),
            resolved_topics={
                "/imu/data": ("/imu/data", ["sensor_msgs/msg/Imu"]),
                "/dvl/data": ("/dvl/data", ["geometry_msgs/msg/TwistStamped"]),
            },
            missing_topics=["/camera/image_raw"],
            type_unresolved_topics=["/status"],
        ),
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.topic_readiness.collect_resolved_topic_payloads_once",
        lambda topics, *, timeout_s, topic_message_types: (
            {"/imu/data": {"orientation": {"x": 0.0}}},
            ["/dvl/data"],
        ),
    )

    report = check_topics_ready(
        ["/imu/data", "/camera/image_raw", "/status", "/dvl/data"],
        timeout_s=0.5,
    )

    assert report.ready is False
    assert report.missing_topics == ["/camera/image_raw"]
    assert report.type_unresolved_topics == ["/status"]
    assert report.silent_topics == ["/dvl/data"]


def test_check_topics_ready_can_temporarily_start_simulation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[str] = []

    class _FakeWorldControlClient:
        def acquire_simulation_hold(self) -> None:
            calls.append("acquire")

        def release_simulation_hold(self) -> None:
            calls.append("release")

    monkeypatch.setattr(
        "mil_robogym.clients.world_control_client.WorldControlClient",
        _FakeWorldControlClient,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.topic_readiness.query_topics_in_graph",
        lambda topics, *, timeout_s: TopicGraphResolution(
            requested_topics=list(topics),
            resolved_topics={
                "/imu/data": ("/imu/data", ["sensor_msgs/msg/Imu"]),
            },
            missing_topics=[],
            type_unresolved_topics=[],
        ),
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.topic_readiness.collect_resolved_topic_payloads_once",
        lambda topics, *, timeout_s, topic_message_types: (
            {"/imu/data": {"orientation": {"x": 0.0}}},
            [],
        ),
    )

    report = check_topics_ready(
        ["/imu/data"],
        timeout_s=0.5,
        start_simulation=True,
    )

    assert report.ready is True
    assert calls == ["acquire", "release"]


def test_ensure_project_input_topics_ready_raises_descriptive_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "mil_robogym.data_collection.topic_readiness.check_topics_ready",
        lambda topics, *, timeout_s, start_simulation: TopicReadinessReport(
            requested_topics=list(topics),
            resolved_topics={"/imu/data": "/imu/data"},
            missing_topics=["/camera/image_raw"],
            type_unresolved_topics=[],
            silent_topics=["/imu/data"],
        ),
    )

    with pytest.raises(TopicReadinessError) as exc_info:
        ensure_project_input_topics_ready(
            {
                "name": "Demo Project",
                "world_file": "robosub_2025.world",
                "model_name": "sub",
                "random_spawn_space": {
                    "enabled": False,
                    "coord1_4d": [0.0, 0.0, 0.0, 0.0],
                    "coord2_4d": [0.0, 0.0, 0.0, 0.0],
                },
                "input_topics": {
                    "/imu/data": ["orientation.x"],
                    "/camera/image_raw": [],
                },
                "output_topics": {},
            },
            timeout_s=0.5,
            operation="start training",
            start_simulation=True,
        )

    assert str(exc_info.value) == (
        "Cannot start training because project input topics are not ready.\n"
        "Missing from ROS 2 graph: ['/camera/image_raw']\n"
        "Not receiving messages within the readiness timeout: ['/imu/data']"
    )
