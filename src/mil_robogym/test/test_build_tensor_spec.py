"""Tests for building tensor specs from sampled project topics."""

import pytest

from mil_robogym.data_collection.build_tensor_spec import build_tensor_spec


def _project(input_topics: list[str], output_topics: list[str]) -> dict:
    """Builds a minimal project payload used by tensor-spec tests."""
    return {
        "project_name": "Tensor Spec Project",
        "world_file": "/tmp/world.sdf",
        "model_name": "model.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": (0.0, 0.0, 0.0, 0.0),
            "coord2_4d": (1.0, 1.0, 1.0, 1.0),
        },
        "input_topics": input_topics,
        "output_topics": output_topics,
    }


def test_build_tensor_spec_success_filters_non_numeric(monkeypatch):
    """Builds tensor features while filtering non-numeric leaves."""

    def fake_sample_project_topics(project, *, timeout_s):
        assert tuple(project["input_topics"]) == ("/imu/data", "/dvl/processed")
        assert tuple(project["output_topics"]) == ("/trajectory/4_deg",)
        return (
            {
                "/imu/data": {
                    "header.stamp.sec": 10,
                    "header.frame_id": "base_link",
                    "orientation.x": 0.1,
                },
                "/dvl/processed": {
                    "velocity.x": -0.2,
                    "is_valid": True,
                },
            },
            {
                "/trajectory/4_deg": {
                    "x": 1.0,
                    "mode": "auto",
                },
            },
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        fake_sample_project_topics,
    )

    result = build_tensor_spec(
        _project(["/imu/data", "/dvl/processed"], ["/trajectory/4_deg"]),
    )

    assert result["input_features"] == [
        "/imu/data:header.stamp.sec",
        "/imu/data:orientation.x",
        "/dvl/processed:velocity.x",
        "/dvl/processed:is_valid",
    ]
    assert result["output_features"] == ["/trajectory/4_deg:x"]
    assert result["input_dim"] == 4
    assert result["output_dim"] == 1
    assert result["ignored_input_features"] == {"/imu/data": ["header.frame_id"]}
    assert result["ignored_output_features"] == {"/trajectory/4_deg": ["mode"]}


def test_build_tensor_spec_preserves_sampler_order(monkeypatch):
    """Preserves topic and field order produced by sampling."""

    def fake_sample_project_topics(project, *, timeout_s):
        assert tuple(project["input_topics"]) == ("/b", "/a")
        assert tuple(project["output_topics"]) == ("/out",)
        return (
            {
                "/b": {
                    "z": 3,
                    "a": 1,
                },
                "/a": {
                    "y": 2,
                },
            },
            {"/out": {"r": 9}},
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        fake_sample_project_topics,
    )

    result = build_tensor_spec(_project(["/b", "/a"], ["/out"]))

    assert result["input_features"] == ["/b:z", "/b:a", "/a:y"]
    assert result["output_features"] == ["/out:r"]


def test_build_tensor_spec_passes_timeout_to_sampling(monkeypatch):
    """Forwards timeout values to project topic sampling."""
    seen = []

    def fake_sample_project_topics(project, *, timeout_s):
        seen.append(
            (
                tuple(project["input_topics"]),
                tuple(project["output_topics"]),
                timeout_s,
            ),
        )
        return (
            {"/in": {"x": 1}},
            {"/out": {"y": 2}},
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        fake_sample_project_topics,
    )

    build_tensor_spec(_project(["/in"], ["/out"]), timeout_s=0.25)

    assert seen == [(("/in",), ("/out",), 0.25)]


def test_build_tensor_spec_strict_numeric_raises(monkeypatch):
    """Raises in strict mode when non-numeric fields are encountered."""

    def fake_sample_project_topics(project, *, timeout_s):
        return (
            {"/in": {"x": 1, "label": "raw"}},
            {"/out": {"y": 2}},
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        fake_sample_project_topics,
    )

    with pytest.raises(RuntimeError, match="non-numeric field"):
        build_tensor_spec(_project(["/in"], ["/out"]), strict_numeric=True)


def test_build_tensor_spec_raises_if_selected_input_has_no_numeric(monkeypatch):
    """Raises when selected inputs produce no numeric tensor features."""

    def fake_sample_project_topics(project, *, timeout_s):
        return (
            {"/in": {"label": "raw"}},
            {"/out": {"y": 2}},
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        fake_sample_project_topics,
    )

    with pytest.raises(RuntimeError, match="no numeric tensor features"):
        build_tensor_spec(_project(["/in"], ["/out"]))


def test_build_tensor_spec_raises_if_selected_output_has_no_numeric(monkeypatch):
    """Raises when selected outputs produce no numeric tensor features."""

    def fake_sample_project_topics(project, *, timeout_s):
        return (
            {"/in": {"x": 1}},
            {"/out": {"label": "raw"}},
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        fake_sample_project_topics,
    )

    with pytest.raises(RuntimeError, match="no numeric tensor features"):
        build_tensor_spec(_project(["/in"], ["/out"]))


def test_build_tensor_spec_allows_empty_topic_lists(monkeypatch):
    """Allows empty input/output topic lists and returns zero dimensions."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        lambda project, *, timeout_s: ({}, {}),
    )

    result = build_tensor_spec(_project([], []))

    assert result == {
        "input_features": [],
        "output_features": [],
        "input_dim": 0,
        "output_dim": 0,
        "ignored_input_features": {},
        "ignored_output_features": {},
    }


def test_build_tensor_spec_raises_on_invalid_timeout(monkeypatch):
    """Rejects zero or negative timeout values."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        lambda project, *, timeout_s: ({}, {}),
    )

    with pytest.raises(ValueError, match="timeout_s must be positive"):
        build_tensor_spec(_project(["/in"], ["/out"]), timeout_s=0.0)


def test_build_tensor_spec_raises_on_duplicate_normalized_input_topics(monkeypatch):
    """Rejects duplicate normalized input topic names."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        lambda project, *, timeout_s: ({}, {}),
    )

    with pytest.raises(ValueError, match="Duplicate normalized input topics"):
        build_tensor_spec(_project(["/imu/data", "imu/data"], ["/out"]))


def test_build_tensor_spec_raises_on_duplicate_normalized_output_topics(monkeypatch):
    """Rejects duplicate normalized output topic names."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        lambda project, *, timeout_s: ({}, {}),
    )

    with pytest.raises(ValueError, match="Duplicate normalized output topics"):
        build_tensor_spec(_project(["/in"], ["/cmd", "cmd"]))


def test_build_tensor_spec_supports_shared_input_output_topic(monkeypatch):
    """Supports a topic selected for both inputs and outputs."""

    def fake_sample_project_topics(project, *, timeout_s):
        assert project["input_topics"] == ["/shared"]
        assert project["output_topics"] == ["/shared"]
        return (
            {"/shared": {"x": 1}},
            {"/shared": {"x": 1}},
        )

    monkeypatch.setattr(
        "mil_robogym.data_collection.build_tensor_spec.sample_project_topics",
        fake_sample_project_topics,
    )

    result = build_tensor_spec(_project(["/shared"], ["/shared"]))

    assert result["input_features"] == ["/shared:x"]
    assert result["output_features"] == ["/shared:x"]
