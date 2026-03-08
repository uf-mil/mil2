"""Tests for topic sampling, ROS type resolution, and flattening behavior."""

import subprocess

import pytest

from mil_robogym.data_collection.sample_input_topics import (
    collect_numeric_values_from_topic_subtopics,
    collect_topic_messages_once,
    numerical_headers_from_topic_subtopics,
    sample_input_topics,
    sample_project_topics,
)


def _project(input_topics: list[str]) -> dict:
    """Builds a minimal project payload used by sampling tests."""
    return {
        "name": "Sampling Test Project",
        "world_file": "/tmp/world.sdf",
        "model_name": "model.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": [0.0, 0.0, 0.0, 0.0],
            "coord2_4d": [1.0, 1.0, 1.0, 1.0],
        },
        "input_topics": {topic: [] for topic in input_topics},
        "output_topics": {},
    }


def test_sample_input_topics_success(monkeypatch):
    """Samples topics successfully and flattens nested/default message fields."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data", "/down_cam/image_raw"],
    )

    topic_types = {
        "/imu/data": "sensor_msgs/msg/Imu\n",
        "/down_cam/image_raw": "sensor_msgs/msg/Image\n",
    }
    default_messages = {
        "sensor_msgs/msg/Imu": {
            "header": {"stamp": {"sec": 10, "nanosec": 20}},
            "orientation": {"x": 0.1, "y": 0.2},
            "covariance": [1.0, 2.0],
        },
        "sensor_msgs/msg/Image": {
            "encoding": "rgb8",
            "data": [17, 42],
        },
    }

    def fake_run(command, **kwargs):
        topic = command[3]
        return subprocess.CompletedProcess(
            args=command,
            returncode=0,
            stdout=topic_types[topic],
            stderr="",
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics._default_message_as_dict",
        lambda msg_type: default_messages[msg_type],
    )

    result = sample_input_topics(_project(["/imu/data", "/down_cam/image_raw"]))

    assert list(result.keys()) == ["/imu/data", "/down_cam/image_raw"]
    assert list(result["/imu/data"].keys()) == [
        "covariance[0]",
        "covariance[1]",
        "header.stamp.nanosec",
        "header.stamp.sec",
        "orientation.x",
        "orientation.y",
    ]
    assert result["/imu/data"]["header.stamp.sec"] == 10
    assert result["/down_cam/image_raw"]["data[1]"] == 42
    assert result["/down_cam/image_raw"]["encoding"] == "rgb8"


def test_sample_input_topics_flattens_nested_message_types(monkeypatch):
    """Flattens nested list/dict message structures into feature keys."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/nested/msg"],
    )

    def fake_run(command, **kwargs):
        return subprocess.CompletedProcess(
            args=command,
            returncode=0,
            stdout="custom_msgs/msg/Nested\n",
            stderr="",
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics._default_message_as_dict",
        lambda _msg_type: {
            "outer": [
                {
                    "inner": {
                        "x": 1.0,
                        "label": "a",
                    },
                },
                {
                    "inner": {
                        "x": 2.0,
                        "label": "b",
                    },
                },
            ],
        },
    )

    result = sample_input_topics(_project(["/nested/msg"]))

    assert result["/nested/msg"]["outer[0].inner.x"] == 1.0
    assert result["/nested/msg"]["outer[1].inner.x"] == 2.0
    assert result["/nested/msg"]["outer[0].inner.label"] == "a"
    assert result["/nested/msg"]["outer[1].inner.label"] == "b"


def test_sample_project_topics_samples_shared_topic_once(monkeypatch):
    """Samples a shared input/output topic once and returns both mappings."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/shared"],
    )

    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(
            args=command,
            returncode=0,
            stdout="std_msgs/msg/Int32\n",
            stderr="",
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics._default_message_as_dict",
        lambda _msg_type: {"x": 1},
    )

    project = _project(["/shared"])
    project["output_topics"] = {"/shared": []}
    sampled_inputs, sampled_outputs = sample_project_topics(project)

    assert len(calls) == 1
    assert sampled_inputs["/shared"]["x"] == 1
    assert sampled_outputs["/shared"]["x"] == 1


def test_sample_input_topics_accepts_topics_without_leading_slash(monkeypatch):
    """Resolves topics even when the request omits the leading slash."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data"],
    )

    seen = []

    def fake_run(command, **kwargs):
        seen.append(command)
        return subprocess.CompletedProcess(
            args=command,
            returncode=0,
            stdout="std_msgs/msg/Int32\n",
            stderr="",
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics._default_message_as_dict",
        lambda _msg_type: {"x": 1},
    )

    result = sample_input_topics(_project(["imu/data"]))

    assert seen[0] == ("ros2", "topic", "type", "/imu/data")
    assert result["imu/data"]["x"] == 1


def test_sample_input_topics_returns_empty_for_empty_input_topics():
    """Returns an empty mapping when no input topics are requested."""
    result = sample_input_topics(_project([]))
    assert result == {}


def test_sample_input_topics_raises_on_invalid_timeout():
    """Raises ValueError when timeout_s is not positive."""
    with pytest.raises(ValueError):
        sample_input_topics(_project(["/imu/data"]), timeout_s=0.0)


def test_sample_input_topics_raises_on_missing_topic(monkeypatch):
    """Raises when requested topics are missing from the ROS graph."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data"],
    )

    with pytest.raises(RuntimeError, match="not found"):
        sample_input_topics(_project(["/imu/data", "/depth"]))


def test_sample_input_topics_raises_when_topic_listing_fails(monkeypatch):
    """Raises when ROS topic discovery fails before sampling."""

    def fake_get_topics():
        raise RuntimeError("ros2 topic list failed")

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        fake_get_topics,
    )

    with pytest.raises(RuntimeError, match="Failed to list ROS 2 topics"):
        sample_input_topics(_project(["/imu/data"]))


def test_sample_input_topics_raises_on_echo_nonzero_return_code(monkeypatch):
    """Raises when `ros2 topic type` returns a non-zero exit code."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data"],
    )

    def fake_run(command, **kwargs):
        return subprocess.CompletedProcess(
            args=command,
            returncode=2,
            stdout="",
            stderr="no publishers",
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )

    with pytest.raises(
        RuntimeError,
        match="Failed to resolve topic type for '/imu/data'",
    ):
        sample_input_topics(_project(["/imu/data"]))


def test_sample_input_topics_raises_on_topic_type_timeout(monkeypatch):
    """Raises when topic type resolution exceeds the timeout."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data"],
    )

    def fake_run(command, **kwargs):
        raise subprocess.TimeoutExpired(cmd=command, timeout=0.05)

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )

    with pytest.raises(RuntimeError, match="Timed out"):
        sample_input_topics(_project(["/imu/data"]), timeout_s=0.05)


def test_sample_input_topics_raises_when_ros2_cli_missing_during_topic_type(
    monkeypatch,
):
    """Raises when the ROS 2 CLI is unavailable during type resolution."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data"],
    )

    def fake_run(command, **kwargs):
        raise FileNotFoundError("ros2 not found")

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )

    with pytest.raises(RuntimeError, match="No ROS 2 CLI found"):
        sample_input_topics(_project(["/imu/data"]))


def test_sample_input_topics_raises_on_empty_topic_type_stdout(monkeypatch):
    """Raises when the topic type command produces empty output."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data"],
    )

    def fake_run(command, **kwargs):
        return subprocess.CompletedProcess(
            args=command,
            returncode=0,
            stdout="  \n",
            stderr="",
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )

    with pytest.raises(RuntimeError, match="empty topic type output"):
        sample_input_topics(_project(["/imu/data"]))


def test_sample_input_topics_raises_on_message_type_resolution_failure(monkeypatch):
    """Propagates failures from default message resolution."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data"],
    )

    def fake_run(command, **kwargs):
        return subprocess.CompletedProcess(
            args=command,
            returncode=0,
            stdout="sensor_msgs/msg/Imu\n",
            stderr="",
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )

    def fake_default_message(_msg_type):
        raise RuntimeError("resolution failed")

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics._default_message_as_dict",
        fake_default_message,
    )

    with pytest.raises(RuntimeError, match="resolution failed"):
        sample_input_topics(_project(["/imu/data"]))


def test_sample_input_topics_flattens_scalar_root(monkeypatch):
    """Stores a scalar root message under the reserved `value` key."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/scalar_topic"],
    )

    def fake_run(command, **kwargs):
        return subprocess.CompletedProcess(
            args=command,
            returncode=0,
            stdout="std_msgs/msg/Int32\n",
            stderr="",
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.subprocess.run",
        fake_run,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics._default_message_as_dict",
        lambda _msg_type: 3,
    )

    result = sample_input_topics(_project(["/scalar_topic"]))
    assert result["/scalar_topic"]["value"] == 3


def test_collect_topic_messages_once_returns_one_payload_per_topic(monkeypatch):
    """Collects one parsed ROS payload per requested topic in order."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data", "/front_cam/image_raw"],
    )

    payloads = {
        "/imu/data": {
            "header": {"stamp": {"sec": 1}},
            "x": 2,
        },
        "/front_cam/image_raw": {
            "height": 600,
            "width": 960,
            "encoding": "rgb8",
            "data": [1, 2],
        },
    }

    def fake_collect(resolved_topics, timeout_s):
        assert timeout_s == 2.0
        return [payloads[topic] for topic in resolved_topics]

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics._collect_topic_messages_once_via_subscriptions",
        fake_collect,
    )

    result = collect_topic_messages_once(["/imu/data", "/front_cam/image_raw"])

    assert len(result) == 2
    assert result[0]["header"]["stamp"]["sec"] == 1
    assert result[1]["encoding"] == "rgb8"
    assert result[1]["data"] == [1, 2]


def test_collect_numeric_values_from_topic_subtopics_uses_header_order(monkeypatch):
    """Builds stable headers and value rows from selected numeric subtopics."""
    topic_subtopics = {
        "/imu/data": ["orientation.x", "orientation.y", "header.frame_id"],
        "/trajectory/4_deg": ["yaw"],
    }
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.collect_topic_messages_once",
        lambda topics, timeout_s=2.0: [
            {
                "orientation": {"x": 0.25, "y": -0.5},
                "header": {"frame_id": "sub9"},
            },
            {"yaw": 1.2},
        ],
    )

    headers = numerical_headers_from_topic_subtopics(topic_subtopics)
    values = collect_numeric_values_from_topic_subtopics(topic_subtopics)

    assert headers == [
        "/imu/data:orientation.x",
        "/imu/data:orientation.y",
        "/imu/data:header.frame_id",
        "/trajectory/4_deg:yaw",
    ]
    assert values == [0.25, -0.5, None, 1.2]
