"""Tests for agent folder creation and timestamp formatting."""

from datetime import datetime
from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.filesystem import (
    _format_agent_timestamp,
    create_agent_folder,
    get_training_project_dir_path,
    get_training_project_dir_paths,
)


def test_format_agent_timestamp():
    """Formats the agent timestamp using the expected 12-hour naming convention."""
    dt = datetime(2026, 2, 22, 23, 0)  # 11:00 pm
    assert _format_agent_timestamp(dt) == "2026_02_22_11_00_pm"


def test_create_agent_folder_creates_files(tmp_path: Path):
    """Creates an agent folder with weights and metrics artifacts."""
    project_dir = tmp_path / "projects" / "start_gate_agent"
    project_dir.mkdir(parents=True)

    model_src = tmp_path / "generator_model.zip"
    model_src.write_bytes(b"fake-weights")

    metrics = {"reward": [0.1, 0.2, 0.3], "loss": [3.0, 2.0, 1.0]}
    dt = datetime(2026, 2, 22, 23, 0)

    agent_dir = create_agent_folder(
        project_dir,
        trained_model_path=model_src,
        training_metrics=metrics,
        num_demos=11,
        created_at=dt,
        model_file_name="generator_model.zip",
        checkpoint_episode=10,
        training_settings={
            "num_episodes": 500,
            "rollout_steps": 2048,
            "generator_learning_rate": 1e-3,
            "discriminator_learning_rate": 3e-3,
            "z_size": 6,
            "e_hidden_size": 128,
            "i_c": 0.5,
            "beta_step_size": 1e-3,
            "gamma": 0.99,
            "save_every": 10,
            "seed": 42,
            "max_step_count": None,
            "expert_noise_std": 1e-4,
        },
    )

    assert agent_dir.name == "2026_02_22_11_00_pm"
    assert (agent_dir / "generator_model.zip").read_bytes() == b"fake-weights"
    config = yaml.safe_load((agent_dir / "config.yaml").read_text(encoding="utf-8"))
    assert config["robogym_agent"]["name"] == "2026_02_22_11_00_pm"
    assert config["robogym_agent"]["num_demos"] == 11
    assert config["robogym_agent"]["model_file"] == "generator_model.zip"
    assert config["robogym_agent"]["checkpoint_episode"] == 10
    assert config["robogym_agent"]["training_settings"]["num_episodes"] == 500
    assert config["robogym_agent"]["training_settings"]["max_step_count"] is None
    assert (agent_dir / "training_metrics.csv").exists()


def test_create_agent_folder_raises_if_exists(tmp_path: Path):
    """Raises when creating an agent folder for an existing timestamp."""
    project_dir = tmp_path / "proj"
    project_dir.mkdir()

    weights_src = tmp_path / "w.pt"
    weights_src.write_bytes(b"x")

    metrics = {"reward": [1.0]}

    dt = datetime(2026, 2, 22, 23, 0)

    create_agent_folder(
        project_dir,
        trained_model_path=weights_src,
        training_metrics=metrics,
        created_at=dt,
    )

    with pytest.raises(FileExistsError):
        create_agent_folder(
            project_dir,
            trained_model_path=weights_src,
            training_metrics=metrics,
            created_at=dt,
        )


def test_get_training_project_dir_paths_prefers_source(tmp_path: Path, monkeypatch):
    """Prefers the source project tree for generated training artifacts."""
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

    project = {"name": "Start Gate Agent"}
    paths = get_training_project_dir_paths(project)

    assert paths == [
        source_projects / "start_gate_agent",
        share_dir / "projects" / "start_gate_agent",
    ]
    assert get_training_project_dir_path(project) == paths[0]
