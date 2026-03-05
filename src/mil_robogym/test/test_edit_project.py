"""Tests for editing project configs across share and source trees."""

from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.filesystem import edit_project


def _project_payload(name: str) -> dict:
    """Builds a minimal project payload for edit tests."""
    return {
        "project_name": name,
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": (0.0, 0.0, 0.0, 0.0),
            "coord2_4d": (1.0, 2.0, 3.0, 4.0),
        },
        "input_topics": ["imu/processed"],
        "output_topics": ["trajectory/4_deg"],
    }


def test_edit_project_updates_share_and_source(tmp_path: Path, monkeypatch):
    """Renames and updates project configs in both share and source directories."""
    share_dir = tmp_path / "install" / "mil_robogym" / "share" / "mil_robogym"
    source_projects = tmp_path / "src" / "mil_robogym" / "projects"
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_package_share_dir",
        lambda: share_dir,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_source_projects_dir",
        lambda _share_dir: source_projects,
    )

    share_old = share_dir / "projects" / "old_name"
    source_old = source_projects / "old_name"
    share_old.mkdir(parents=True)
    source_old.mkdir(parents=True)
    (share_old / "demos").mkdir()
    (source_old / "demos").mkdir()
    (share_old / "config.yaml").write_text(
        "robogym_project:\n  name: old\n",
        encoding="utf-8",
    )
    (source_old / "config.yaml").write_text(
        "robogym_project:\n  name: old\n",
        encoding="utf-8",
    )

    updated = _project_payload("New Name")
    edited_dir = edit_project(updated, original_project_name="Old Name")

    share_new = share_dir / "projects" / "new_name"
    source_new = source_projects / "new_name"
    assert edited_dir == share_new
    assert share_new.is_dir()
    assert source_new.is_dir()
    assert not share_old.exists()
    assert not source_old.exists()

    share_cfg = yaml.safe_load((share_new / "config.yaml").read_text(encoding="utf-8"))
    source_cfg = yaml.safe_load(
        (source_new / "config.yaml").read_text(encoding="utf-8"),
    )
    assert share_cfg["robogym_project"]["project_name"] == "New Name"
    assert source_cfg["robogym_project"]["project_name"] == "New Name"
    assert share_cfg == source_cfg


def test_edit_project_raises_if_share_project_missing(tmp_path: Path, monkeypatch):
    """Raises when trying to edit a project absent from the share directory."""
    share_dir = tmp_path / "install" / "mil_robogym" / "share" / "mil_robogym"
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_package_share_dir",
        lambda: share_dir,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_source_projects_dir",
        lambda _share_dir: None,
    )

    with pytest.raises(FileNotFoundError, match="Project folder does not exist"):
        edit_project(_project_payload("Missing"))
