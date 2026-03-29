"""Tests for loading and validating project configuration files."""

from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.get_all_project_config import (
    find_projects_dir,
    get_all_project_config,
)


def _write_project_config(base_dir: Path, payload: dict) -> None:
    """Writes a project config file under a project directory."""
    base_dir.mkdir(parents=True, exist_ok=True)
    (base_dir / "config.yaml").write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding="utf-8",
    )


def test_get_all_project_config_returns_new_schema_with_num_demos(
    tmp_path: Path,
    monkeypatch,
):
    """Loads project configs that use name + topic-subtopics mappings."""
    projects_dir = tmp_path / "projects"
    project_dir = projects_dir / "example_project"
    (project_dir / "demos" / "demo_1").mkdir(parents=True, exist_ok=True)
    _write_project_config(
        project_dir,
        {
            "robogym_project": {
                "name": "Example Project",
                "world_file": "src/default/world/file",
                "model_name": "weights.pt",
                "random_spawn_space": {
                    "enabled": False,
                    "coord1_4d": [0.0, 0.0, 0.0, 0.0],
                    "coord2_4d": [1.0, 2.0, 3.0, 4.0],
                },
                "input_topics": {"/imu/data": ["orientation.x"]},
                "output_topics": {"/trajectory/4_deg": ["yaw"]},
            },
        },
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.get_all_project_config.find_projects_dir",
        lambda: projects_dir,
    )

    configs = get_all_project_config()

    assert len(configs) == 1
    assert configs[0]["robogym_project"]["name"] == "Example Project"
    assert configs[0]["robogym_project"]["input_topics"]["/imu/data"] == [
        "orientation.x",
    ]
    assert configs[0]["num_demos"] == 1


def test_get_all_project_config_raises_on_project_name_key(tmp_path: Path, monkeypatch):
    """Rejects project_name key now that name is the ground-truth schema key."""
    projects_dir = tmp_path / "projects"
    _write_project_config(
        projects_dir / "legacy_project",
        {
            "robogym_project": {
                "project_name": "Legacy Project",
                "input_topics": {},
                "output_topics": {},
            },
        },
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.get_all_project_config.find_projects_dir",
        lambda: projects_dir,
    )

    with pytest.raises(ValueError, match="robogym_project\\.name"):
        get_all_project_config()


def test_find_projects_dir_creates_missing_projects_dir(tmp_path: Path, monkeypatch):
    """Creates the projects directory when it has been manually deleted."""
    source_projects = tmp_path / "src" / "mil_robogym" / "projects"
    monkeypatch.setattr(
        "mil_robogym.data_collection.get_all_project_config.resolve_source_projects_dir",
        lambda package_name="mil_robogym": source_projects,
    )

    projects_dir = find_projects_dir()

    assert projects_dir == source_projects
    assert projects_dir.is_dir()
