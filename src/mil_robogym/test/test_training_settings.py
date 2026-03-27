"""Tests for training-settings defaults, normalization, and persistence."""

from pathlib import Path

import yaml

from mil_robogym.data_collection.filesystem import (
    create_project_folder,
    update_project_training_settings,
)
from mil_robogym.data_collection.get_all_project_config import get_all_project_config
from mil_robogym.vairl.training_settings import (
    get_default_training_settings,
    normalize_training_settings,
)


def _project_payload(name: str) -> dict:
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


def test_normalize_training_settings_merges_defaults():
    """Missing settings fall back to defaults while overrides are preserved."""
    normalized = normalize_training_settings(
        {
            "num_episodes": "25",
            "rollout_steps": "1024",
            "max_step_count": "",
        },
    )

    assert normalized["num_episodes"] == 25
    assert normalized["rollout_steps"] == 1024
    assert normalized["max_step_count"] is None
    assert normalized["save_every"] == get_default_training_settings()["save_every"]


def test_update_project_training_settings_writes_config_and_loader_returns_it(
    tmp_path: Path,
    monkeypatch,
):
    """Project config keeps a top-level robogym_training block after updates."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_package_share_dir",
        lambda: tmp_path,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.get_all_project_config.find_projects_dir",
        lambda: tmp_path / "projects",
    )

    project = _project_payload("Settings Project")
    project_dir = create_project_folder(project)

    updated_settings = normalize_training_settings(
        {
            "num_episodes": 123,
            "rollout_steps": 456,
            "save_every": 7,
            "seed": 99,
        },
    )
    updated_paths = update_project_training_settings(project, updated_settings)

    assert updated_paths == [project_dir / "config.yaml"]

    parsed = yaml.safe_load((project_dir / "config.yaml").read_text(encoding="utf-8"))
    assert parsed["robogym_training"]["num_episodes"] == 123
    assert parsed["robogym_training"]["rollout_steps"] == 456
    assert parsed["robogym_training"]["save_every"] == 7
    assert parsed["robogym_project"]["name"] == "Settings Project"

    loaded = get_all_project_config()
    assert len(loaded) == 1
    assert loaded[0]["robogym_training"]["num_episodes"] == 123
