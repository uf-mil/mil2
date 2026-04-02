"""Tests for deleting saved agent folders."""

from __future__ import annotations

from pathlib import Path

import yaml

from mil_robogym.data_collection.delete_saved_agent import (
    delete_saved_agent_artifacts,
)


def _write_saved_agent(project_dir: Path, agent_name: str) -> Path:
    agent_dir = project_dir / "agents" / agent_name
    agent_dir.mkdir(parents=True, exist_ok=True)
    (agent_dir / "config.yaml").write_text(
        yaml.safe_dump(
            {
                "robogym_agent": {
                    "name": agent_name,
                    "num_demos": 3,
                    "model_file": "generator_model.zip",
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (agent_dir / "generator_model.zip").write_bytes(b"fake-model")
    return agent_dir


def test_delete_saved_agent_artifacts_removes_all_training_artifact_copies(
    tmp_path: Path,
    monkeypatch,
):
    """Deletes the same saved agent from every training artifact root."""
    source_project_dir = tmp_path / "source_project"
    mirror_project_dir = tmp_path / "mirror_project"
    agent_name = "2026_04_02_11_45_am_final"

    source_agent_dir = _write_saved_agent(source_project_dir, agent_name)
    mirror_agent_dir = _write_saved_agent(mirror_project_dir, agent_name)

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent.get_training_project_dir_path",
        lambda _project: source_project_dir,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.delete_saved_agent.get_training_project_dir_paths",
        lambda _project: [source_project_dir, mirror_project_dir],
    )

    deleted_paths = delete_saved_agent_artifacts(
        {"robogym_project": {"name": "Demo Project"}},
        agent_name,
    )

    assert deleted_paths == [source_agent_dir, mirror_agent_dir]
    assert not source_agent_dir.exists()
    assert not mirror_agent_dir.exists()
