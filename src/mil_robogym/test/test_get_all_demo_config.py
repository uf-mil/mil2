"""Tests for loading and validating demo configuration files."""

from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.get_all_demo_config import get_all_demo_config


def _write_demo_config(
    base_dir: Path,
    *,
    folder_name: str,
    name: str,
    sampling_rate: float = 10.0,
) -> None:
    """Writes a demo config file under a demo subdirectory."""
    demo_dir = base_dir / folder_name
    demo_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "robogym_demo": {
            "name": name,
            "start_position": [0.0, 0.0, 0.0, 0.0],
            "sampling_rate": sampling_rate,
        },
    }
    (demo_dir / "config.yaml").write_text(yaml.safe_dump(payload), encoding="utf-8")


def test_get_all_demo_config_returns_configs_by_raw_demo_name(
    tmp_path: Path,
    monkeypatch,
):
    """Loads all demo configs keyed by their raw demo names."""
    projects_dir = tmp_path / "projects"
    demos_dir = projects_dir / "example_project" / "demos"
    _write_demo_config(demos_dir, folder_name="demo_1", name="Demo One")
    _write_demo_config(demos_dir, folder_name="demo_2", name="Demo Two")
    monkeypatch.setattr(
        "mil_robogym.data_collection.get_all_demo_config.find_projects_dir",
        lambda: projects_dir,
    )

    configs = get_all_demo_config("Example Project")

    assert set(configs.keys()) == {"Demo One", "Demo Two"}
    assert configs["Demo One"]["robogym_demo"]["sampling_rate"] == 10.0


def test_get_all_demo_config_raises_on_invalid_robogym_demo_structure(
    tmp_path: Path,
    monkeypatch,
):
    """Raises when robogym_demo is present but not a mapping."""
    projects_dir = tmp_path / "projects"
    demo_dir = projects_dir / "example_project" / "demos" / "broken"
    demo_dir.mkdir(parents=True, exist_ok=True)
    (demo_dir / "config.yaml").write_text(
        yaml.safe_dump({"robogym_demo": ["not-a-dict"]}),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.get_all_demo_config.find_projects_dir",
        lambda: projects_dir,
    )

    with pytest.raises(ValueError, match="invalid robogym_demo structure"):
        get_all_demo_config("Example Project")


def test_get_all_demo_config_returns_empty_when_demos_dir_missing(
    tmp_path: Path,
    monkeypatch,
):
    """Creates demos directory and returns empty configs when none exist yet."""
    projects_dir = tmp_path / "projects"
    (projects_dir / "example_project").mkdir(parents=True, exist_ok=True)
    monkeypatch.setattr(
        "mil_robogym.data_collection.get_all_demo_config.find_projects_dir",
        lambda: projects_dir,
    )

    configs = get_all_demo_config("Example Project")

    assert configs == {}
    assert (projects_dir / "example_project" / "demos").is_dir()


def test_get_all_demo_config_raises_when_name_missing(tmp_path: Path, monkeypatch):
    """Raises when robogym_demo is missing the name field."""
    projects_dir = tmp_path / "projects"
    demo_dir = projects_dir / "example_project" / "demos" / "broken"
    demo_dir.mkdir(parents=True, exist_ok=True)
    (demo_dir / "config.yaml").write_text(
        yaml.safe_dump({"robogym_demo": {"sampling_rate": 5.0}}),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.get_all_demo_config.find_projects_dir",
        lambda: projects_dir,
    )

    with pytest.raises(ValueError, match="invalid robogym_demo structure"):
        get_all_demo_config("Example Project")
