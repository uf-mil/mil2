"""Tests for loading saved agent artifacts."""

from __future__ import annotations

from pathlib import Path

import yaml

from mil_robogym.data_collection.load_saved_agent import load_saved_agent


def _write_saved_agent(
    project_dir: Path,
    *,
    agent_name: str,
    model_file_name: str = "generator_model.zip",
    checkpoint_episode: int | None = None,
    include_metrics_csv: bool = True,
) -> Path:
    agent_dir = project_dir / "agents" / agent_name
    agent_dir.mkdir(parents=True, exist_ok=True)

    config = {
        "robogym_agent": {
            "name": agent_name,
            "num_demos": 4,
            "model_file": model_file_name,
            "training_settings": {
                "num_episodes": 20,
                "rollout_steps": 512,
                "save_every": 5,
            },
        },
    }
    if checkpoint_episode is not None:
        config["robogym_agent"]["checkpoint_episode"] = checkpoint_episode

    (agent_dir / "config.yaml").write_text(
        yaml.safe_dump(config, sort_keys=False),
        encoding="utf-8",
    )
    (agent_dir / model_file_name).write_bytes(b"fake-model")

    if include_metrics_csv:
        (agent_dir / "training_metrics.csv").write_text(
            "episode,reward_mean\n1,0.5\n",
            encoding="utf-8",
        )

    return agent_dir


def test_load_saved_agent_reads_valid_saved_agent(tmp_path: Path, monkeypatch):
    """Returns a validated handle for a saved agent folder."""
    project_dir = tmp_path / "demo_project"
    agent_dir = _write_saved_agent(
        project_dir,
        agent_name="2026_03_30_11_15_am_ep_0005",
        checkpoint_episode=5,
    )

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent.get_training_project_dir_path",
        lambda _project: project_dir,
    )

    handle = load_saved_agent(
        {"robogym_project": {"name": "Demo Project"}},
        "2026_03_30_11_15_am_ep_0005",
    )

    assert handle.project_name == "Demo Project"
    assert handle.agent_name == "2026_03_30_11_15_am_ep_0005"
    assert handle.agent_dir == agent_dir
    assert handle.model_path == agent_dir / "generator_model.zip"
    assert handle.metrics_csv_path == agent_dir / "training_metrics.csv"
    assert handle.model_file_name == "generator_model.zip"
    assert handle.num_demos == 4
    assert handle.checkpoint_episode == 5
    assert handle.is_final is False
    assert handle.training_settings is not None
    assert handle.training_settings["num_episodes"] == 20


def test_load_saved_agent_rejects_missing_model_file(tmp_path: Path, monkeypatch):
    """Raises a clear error if the saved model file is missing."""
    project_dir = tmp_path / "demo_project"
    agent_dir = _write_saved_agent(project_dir, agent_name="saved_final")
    (agent_dir / "generator_model.zip").unlink()

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent.get_training_project_dir_path",
        lambda _project: project_dir,
    )

    try:
        load_saved_agent({"name": "Demo Project"}, "saved_final")
    except FileNotFoundError as exc:
        assert "Saved model file does not exist" in str(exc)
    else:
        raise AssertionError("Expected FileNotFoundError for missing model file.")


def test_load_saved_agent_rejects_unsafe_model_paths(tmp_path: Path, monkeypatch):
    """Rejects model_file config values that escape the agent directory."""
    project_dir = tmp_path / "demo_project"
    _write_saved_agent(
        project_dir,
        agent_name="saved_final",
        model_file_name="../outside.zip",
    )

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent.get_training_project_dir_path",
        lambda _project: project_dir,
    )

    try:
        load_saved_agent({"name": "Demo Project"}, "saved_final")
    except ValueError as exc:
        assert "must stay inside the agent folder" in str(exc)
    else:
        raise AssertionError("Expected ValueError for unsafe model path.")
