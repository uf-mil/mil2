"""Tests for saved agent config discovery."""

from __future__ import annotations

from pathlib import Path

import yaml

from mil_robogym.data_collection.get_all_agent_config import get_all_agent_config


def _write_agent_config(agent_dir: Path, *, num_demos: int) -> None:
    agent_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "robogym_agent": {
            "name": agent_dir.name,
            "num_demos": num_demos,
        },
    }
    (agent_dir / "config.yaml").write_text(
        yaml.safe_dump(payload),
        encoding="utf-8",
    )


def test_get_all_agent_config_reads_valid_agent_configs(tmp_path: Path):
    """Loads saved agents from a project directory."""
    project_dir = tmp_path / "project"
    _write_agent_config(project_dir / "agents" / "agent_a", num_demos=3)
    _write_agent_config(project_dir / "agents" / "agent_b", num_demos=7)

    configs = get_all_agent_config(project_dir)

    assert configs == {
        "agent_a": {"num_demos": 3},
        "agent_b": {"num_demos": 7},
    }


def test_get_all_agent_config_skips_invalid_agent_folders(tmp_path: Path):
    """Ignores partial or malformed agent folders while keeping valid ones."""
    project_dir = tmp_path / "project"
    _write_agent_config(project_dir / "agents" / "valid_agent", num_demos=5)

    invalid_agent = project_dir / "agents" / "missing_config"
    invalid_agent.mkdir(parents=True, exist_ok=True)

    malformed_agent = project_dir / "agents" / "bad_config"
    malformed_agent.mkdir(parents=True, exist_ok=True)
    (malformed_agent / "config.yaml").write_text("[]\n", encoding="utf-8")

    configs = get_all_agent_config(project_dir)

    assert configs == {"valid_agent": {"num_demos": 5}}
