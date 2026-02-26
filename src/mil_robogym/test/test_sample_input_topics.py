import subprocess

import pytest

from mil_robogym.data_collection.sample_input_topics import sample_input_topics


def _project(input_topics: list[str]) -> dict:
    return {
        "project_name": "Sampling Test Project",
        "world_file": "/tmp/world.sdf",
        "model_name": "model.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": (0.0, 0.0, 0.0, 0.0),
            "coord2_4d": (1.0, 1.0, 1.0, 1.0),
        },
        "input_topics": input_topics,
        "output_topics": [],
    }


def test_sample_input_topics_success(monkeypatch):
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


def test_sample_input_topics_accepts_topics_without_leading_slash(monkeypatch):
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
    result = sample_input_topics(_project([]))
    assert result == {}


def test_sample_input_topics_raises_on_invalid_timeout():
    with pytest.raises(ValueError):
        sample_input_topics(_project(["/imu/data"]), timeout_s=0.0)


def test_sample_input_topics_raises_on_missing_topic(monkeypatch):
    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        lambda: ["/imu/data"],
    )

    with pytest.raises(RuntimeError, match="not found"):
        sample_input_topics(_project(["/imu/data", "/depth"]))


def test_sample_input_topics_raises_when_topic_listing_fails(monkeypatch):
    def fake_get_topics():
        raise RuntimeError("ros2 topic list failed")

    monkeypatch.setattr(
        "mil_robogym.data_collection.sample_input_topics.get_ros2_topics",
        fake_get_topics,
    )

    with pytest.raises(RuntimeError, match="Failed to list ROS 2 topics"):
        sample_input_topics(_project(["/imu/data"]))


def test_sample_input_topics_raises_on_echo_nonzero_return_code(monkeypatch):
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
