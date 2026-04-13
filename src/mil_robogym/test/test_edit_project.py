"""Tests for editing project configs in the source tree."""

from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.filesystem import edit_project


def _project_payload(name: str) -> dict:
    """Builds a minimal project payload for edit tests."""
    return {
        "name": name,
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": [0.0, 0.0, 0.0, 0.0],
            "coord2_4d": [1.0, 2.0, 3.0, 4.0],
        },
        "input_topics": {"imu/processed": ["orientation.x"]},
        "output_topics": {"trajectory/4_deg": ["yaw"]},
    }


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
    (source_old / "config.yaml").write_text(
        "robogym_project:\n"
        "  name: Old Name\n"
        "  world_file: src/default/world/file\n"
        "  model_name: weights.pt\n"
        "  random_spawn_space:\n"
        "    enabled: false\n"
        "    coord1_4d: [0.0, 0.0, 0.0, 0.0]\n"
        "    coord2_4d: [0.0, 0.0, 0.0, 0.0]\n"
        "  input_topics: {}\n"
        "  output_topics: {}\n"
        "robogym_training:\n"
        "  num_episodes: 77\n"
        "  rollout_steps: 333\n",
        encoding="utf-8",
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
    (source_project / "config.yaml").write_text(
        "robogym_project:\n"
        "  name: Typed Topics Project\n"
        "  world_file: src/default/world/file\n"
        "  model_name: weights.pt\n"
        "  random_spawn_space:\n"
        "    enabled: false\n"
        "    coord1_4d: [0.0, 0.0, 0.0, 0.0]\n"
        "    coord2_4d: [0.0, 0.0, 0.0, 0.0]\n"
        "  input_topics:\n"
        "    camera/image_raw: []\n"
        "  output_topics:\n"
        "    detections: []\n",
        encoding="utf-8",
    )

    updated = _project_payload("Typed Topics Project")
    updated["input_topics"] = {"camera/image_raw": []}
    updated["output_topics"] = {"detections": []}
    updated["input_non_numeric_topics"] = {
        "camera/image_raw": [
            {
                "field_path": "data",
                "data_type": "image",
                "ros_type": "sensor_msgs/msg/Image",
            },
        ],
    }
    updated["output_non_numeric_topics"] = {
        "detections": [
            {
                "field_path": "detections",
                "data_type": "unordered_set",
                "ros_type": "sequence<vision_msgs/msg/Detection2D>",
            },
        ],
    }

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
    (source_project / "config.yaml").write_text(
        "robogym_project:\n"
        "  name: Locked Project\n"
        "  world_file: src/default/world/file\n"
        "  model_name: weights.pt\n"
        "  random_spawn_space:\n"
        "    enabled: false\n"
        "    coord1_4d: [0.0, 0.0, 0.0, 0.0]\n"
        "    coord2_4d: [0.0, 0.0, 0.0, 0.0]\n"
        "  input_topics:\n"
        "    imu/processed: [orientation.x]\n"
        "  output_topics:\n"
        "    trajectory/4_deg: [yaw]\n",
        encoding="utf-8",
    )

    updated = _project_payload("Locked Project")
    updated["input_topics"] = {"imu/processed": ["orientation.y"]}

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
    (source_project / "config.yaml").write_text(
        "robogym_project:\n"
        "  name: Stable Project\n"
        "  world_file: src/default/world/file\n"
        "  model_name: weights.pt\n"
        "  random_spawn_space:\n"
        "    enabled: false\n"
        "    coord1_4d: [0.0, 0.0, 0.0, 0.0]\n"
        "    coord2_4d: [0.0, 0.0, 0.0, 0.0]\n"
        "  input_topics:\n"
        "    imu/processed: [orientation.x]\n"
        "  output_topics:\n"
        "    trajectory/4_deg: [yaw]\n",
        encoding="utf-8",
    )

    updated = _project_payload("Stable Project Renamed")

    edited_dir = edit_project(updated, original_project_name="Stable Project")

    assert edited_dir == source_projects / "stable_project_renamed"
    assert (edited_dir / "demos" / "demo_1").is_dir()
