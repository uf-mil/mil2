"""Tests for editing project configs in the source tree."""

from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.filesystem import edit_project


def _tensor_spec(
    *,
    input_topics: dict[str, list[str]],
    output_topics: dict[str, list[str]],
) -> dict[str, object]:
    input_features = [
        f"{topic}:{field}" for topic, fields in input_topics.items() for field in fields
    ]
    output_features = [
        f"{topic}:{field}"
        for topic, fields in output_topics.items()
        for field in fields
    ]
    return {
        "input_features": input_features,
        "output_features": output_features,
        "input_dim": len(input_features),
        "output_dim": len(output_features),
    }


def _project_payload(
    name: str,
    *,
    input_topics: dict[str, list[str]] | None = None,
    output_topics: dict[str, list[str]] | None = None,
    input_non_numeric_topics: dict[str, list[dict[str, str]]] | None = None,
    output_non_numeric_topics: dict[str, list[dict[str, str]]] | None = None,
) -> dict:
    """Builds a valid project payload for edit tests."""
    resolved_input_topics = input_topics or {"imu/processed": ["orientation.x"]}
    resolved_output_topics = output_topics or {"trajectory/4_deg": ["yaw"]}

    payload = {
        "name": name,
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": [0.0, 0.0, 0.0, 0.0],
            "coord2_4d": [1.0, 2.0, 3.0, 4.0],
        },
        "input_topics": resolved_input_topics,
        "output_topics": resolved_output_topics,
        "tensor_spec": _tensor_spec(
            input_topics=resolved_input_topics,
            output_topics=resolved_output_topics,
        ),
    }
    if input_non_numeric_topics is not None:
        payload["input_non_numeric_topics"] = input_non_numeric_topics
    if output_non_numeric_topics is not None:
        payload["output_non_numeric_topics"] = output_non_numeric_topics
    return payload


def _write_project_config(
    project_dir: Path,
    *,
    project: dict,
    training_settings: dict[str, int] | None = None,
) -> None:
    config = {"robogym_project": project}
    if training_settings is not None:
        config["robogym_training"] = training_settings
    (project_dir / "config.yaml").write_text(
        yaml.safe_dump(config, sort_keys=False),
        encoding="utf-8",
    )


def test_edit_project_updates_source(tmp_path: Path, monkeypatch):
    """Renames and updates project configs in the source directory."""
    source_projects = tmp_path / "src" / "mil_robogym" / "projects"
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_source_projects_dir",
        lambda: source_projects,
    )

    source_old = source_projects / "old_name"
    source_old.mkdir(parents=True)
    (source_old / "demos").mkdir()
    _write_project_config(
        source_old,
        project=_project_payload("Old Name", input_topics={}, output_topics={}),
        training_settings={"num_episodes": 77, "rollout_steps": 333},
    )

    updated = _project_payload("New Name")
    edited_dir = edit_project(updated, original_project_name="Old Name")

    source_new = source_projects / "new_name"
    assert edited_dir == source_new
    assert source_new.is_dir()
    assert not source_old.exists()

    source_cfg = yaml.safe_load(
        (source_new / "config.yaml").read_text(encoding="utf-8"),
    )
    assert source_cfg["robogym_project"]["name"] == "New Name"
    assert source_cfg["robogym_project"]["input_topics"]["imu/processed"] == [
        "orientation.x",
    ]
    assert source_cfg["robogym_project"]["output_topics"]["trajectory/4_deg"] == [
        "yaw",
    ]
    assert source_cfg["robogym_training"]["num_episodes"] == 77
    assert source_cfg["robogym_training"]["rollout_steps"] == 333


def test_edit_project_raises_if_project_missing(tmp_path: Path, monkeypatch):
    """Raises when trying to edit a project absent from the source directory."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_source_projects_dir",
        lambda: tmp_path / "src" / "mil_robogym" / "projects",
    )

    with pytest.raises(FileNotFoundError, match="Project folder does not exist"):
        edit_project(_project_payload("Missing"))


def test_edit_project_preserves_non_numeric_topic_selections(
    tmp_path: Path,
    monkeypatch,
):
    """Writes the new non-numeric topic selection mappings when present."""
    source_projects = tmp_path / "src" / "mil_robogym" / "projects"
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_source_projects_dir",
        lambda: source_projects,
    )

    source_project = source_projects / "typed_topics_project"
    source_project.mkdir(parents=True)
    (source_project / "demos").mkdir()
    _write_project_config(
        source_project,
        project=_project_payload(
            "Typed Topics Project",
            input_topics={"camera/image_raw": []},
            output_topics={"detections": []},
        ),
    )

    updated = _project_payload(
        "Typed Topics Project",
        input_topics={"camera/image_raw": []},
        output_topics={"detections": []},
        input_non_numeric_topics={
            "camera/image_raw": [
                {
                    "field_path": "data",
                    "data_type": "image",
                    "ros_type": "sensor_msgs/msg/Image",
                },
            ],
        },
        output_non_numeric_topics={
            "detections": [
                {
                    "field_path": "detections",
                    "data_type": "unordered_set",
                    "ros_type": "sequence<vision_msgs/msg/Detection2D>",
                },
            ],
        },
    )

    edited_dir = edit_project(updated, original_project_name="Typed Topics Project")
    source_cfg = yaml.safe_load(
        (edited_dir / "config.yaml").read_text(encoding="utf-8"),
    )

    assert source_cfg["robogym_project"]["input_non_numeric_topics"] == {
        "camera/image_raw": [
            {
                "field_path": "data",
                "data_type": "image",
                "ros_type": "sensor_msgs/msg/Image",
            },
        ],
    }
    assert source_cfg["robogym_project"]["output_non_numeric_topics"] == {
        "detections": [
            {
                "field_path": "detections",
                "data_type": "unordered_set",
                "ros_type": "sequence<vision_msgs/msg/Detection2D>",
            },
        ],
    }


def test_edit_project_rejects_topic_changes_when_demos_exist(
    tmp_path: Path,
    monkeypatch,
):
    """Rejects topic edits once a project already has recorded demos."""
    source_projects = tmp_path / "src" / "mil_robogym" / "projects"
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_source_projects_dir",
        lambda: source_projects,
    )

    source_project = source_projects / "locked_project"
    (source_project / "demos" / "demo_1").mkdir(parents=True)
    _write_project_config(
        source_project,
        project=_project_payload("Locked Project"),
    )

    updated = _project_payload(
        "Locked Project",
        input_topics={"imu/processed": ["orientation.y"]},
    )

    with pytest.raises(
        ValueError,
        match="Cannot change project topic selections while demos already exist",
    ):
        edit_project(updated, original_project_name="Locked Project")


def test_edit_project_allows_non_topic_changes_when_demos_exist(
    tmp_path: Path,
    monkeypatch,
):
    """Allows renames when topic selections stay unchanged, even with demos."""
    source_projects = tmp_path / "src" / "mil_robogym" / "projects"
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_source_projects_dir",
        lambda: source_projects,
    )

    source_project = source_projects / "stable_project"
    (source_project / "demos" / "demo_1").mkdir(parents=True)
    _write_project_config(
        source_project,
        project=_project_payload("Stable Project"),
    )

    updated = _project_payload("Stable Project Renamed")

    edited_dir = edit_project(updated, original_project_name="Stable Project")

    assert edited_dir == source_projects / "stable_project_renamed"
    assert (edited_dir / "demos" / "demo_1").is_dir()
