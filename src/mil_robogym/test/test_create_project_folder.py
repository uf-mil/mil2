"""Tests for project folder creation and project-name normalization helpers."""

from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.filesystem import create_project_folder
from mil_robogym.data_collection.utils import to_lower_snake_case


def test_create_project_folder(tmp_path: Path, monkeypatch):
    """Creates a project folder and writes a project config file."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_package_share_dir",
        lambda: tmp_path,
    )

    proj = {
        "name": "Start Gate Agent",
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": [0.0, 0.0, 0.0, 0.0],
            "coord2_4d": [1.0, 2.0, 3.0, 4.0],
        },
        "input_topics": {
            "imu/processed": ["orientation.x", "orientation.y"],
            "dvl/processed": ["velocity.x"],
        },
        "output_topics": {"trajectory/4_deg": ["yaw"]},
    }

    project_dir = create_project_folder(proj)

    assert project_dir.exists()
    assert project_dir.name == "start_gate_agent"

    cfg = (project_dir / "config.yaml").read_text()
    assert "robogym_project" in cfg
    assert "Start Gate Agent" in cfg
    assert "imu/processed" in cfg

    parsed = yaml.safe_load(cfg)
    assert parsed["robogym_project"]["name"] == "Start Gate Agent"
    assert parsed["robogym_project"]["input_topics"]["imu/processed"] == [
        "orientation.x",
        "orientation.y",
    ]
    assert parsed["robogym_project"]["output_topics"]["trajectory/4_deg"] == [
        "yaw",
    ]


def test_to_lower_snake_case_basic():
    """Converts a spaced name into lower snake case."""
    assert to_lower_snake_case("Start Gate Agent") == "start_gate_agent"


def test_to_lower_snake_case_hyphens_and_spaces():
    """Normalizes repeated spaces and hyphens into underscores."""
    assert to_lower_snake_case("Start-Gate agent") == "start_gate_agent"
    assert to_lower_snake_case("Start   Gate---Agent") == "start_gate_agent"


def test_to_lower_snake_case_punctuation():
    """Removes punctuation while preserving the expected word boundaries."""
    assert to_lower_snake_case("Start Gate: Agent!") == "start_gate_agent"
    assert to_lower_snake_case("  Start (Gate) Agent  ") == "start_gate_agent"


def test_to_lower_snake_case_already_snake():
    """Leaves an already normalized snake case string unchanged."""
    assert to_lower_snake_case("start_gate_agent") == "start_gate_agent"


def test_create_project_folder_raises_if_exists(tmp_path, monkeypatch):
    """Raises when creating the same project folder twice."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_package_share_dir",
        lambda: tmp_path,
    )

    proj = {
        "name": "Start Gate Agent",
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": [0.0, 0.0, 0.0, 0.0],
            "coord2_4d": [0.0, 0.0, 0.0, 0.0],
        },
        "input_topics": {"imu/processed": [], "dvl/processed": []},
        "output_topics": {"trajectory/4_deg": []},
    }

    # Create the folder first time
    first = create_project_folder(proj)
    assert first.exists()

    # Second time should throw
    with pytest.raises(FileExistsError):
        create_project_folder(proj)


def test_create_project_folder_writes_tensor_spec(tmp_path: Path, monkeypatch):
    """Persists tensor_spec fields when they are provided in the project payload."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_package_share_dir",
        lambda: tmp_path,
    )

    proj = {
        "name": "Tensor Spec Project",
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": [0.0, 0.0, 0.0, 0.0],
            "coord2_4d": [1.0, 2.0, 3.0, 4.0],
        },
        "input_topics": {"imu/processed": ["x", "y"]},
        "output_topics": {"trajectory/4_deg": ["heading"]},
        "tensor_spec": {
            "input_features": ["imu/processed:x", "imu/processed:y"],
            "output_features": ["trajectory/4_deg:heading"],
            "input_dim": 2,
            "output_dim": 1,
            "ignored_input_features": {"imu/processed": ["frame_id"]},
            "ignored_output_features": {},
        },
    }

    project_dir = create_project_folder(proj)
    cfg = yaml.safe_load((project_dir / "config.yaml").read_text(encoding="utf-8"))

    assert cfg["robogym_project"]["tensor_spec"]["input_dim"] == 2
    assert cfg["robogym_project"]["tensor_spec"]["output_dim"] == 1
    assert cfg["robogym_project"]["tensor_spec"]["input_features"] == [
        "imu/processed:x",
        "imu/processed:y",
    ]
    assert cfg["robogym_project"]["input_topics"]["imu/processed"] == [
        "x",
        "y",
    ]
    assert cfg["robogym_project"]["output_topics"]["trajectory/4_deg"] == [
        "heading",
    ]
    assert "ignored_input_features" not in cfg["robogym_project"]["tensor_spec"]
    assert "ignored_output_features" not in cfg["robogym_project"]["tensor_spec"]


def test_create_project_folder_writes_source_and_install(tmp_path: Path, monkeypatch):
    """Writes identical project config files to install and source project roots."""
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

    proj = {
        "name": "Dual Write Project",
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": [0.0, 0.0, 0.0, 0.0],
            "coord2_4d": [1.0, 2.0, 3.0, 4.0],
        },
        "input_topics": {"imu/processed": ["velocity.x"]},
        "output_topics": {"trajectory/4_deg": ["yaw"]},
    }

    share_project_dir = create_project_folder(proj)
    source_project_dir = source_projects / "dual_write_project"

    assert share_project_dir == share_dir / "projects" / "dual_write_project"
    assert (share_project_dir / "config.yaml").is_file()
    assert (source_project_dir / "config.yaml").is_file()
    assert (share_project_dir / "config.yaml").read_text(
        encoding="utf-8",
    ) == source_project_dir.joinpath(
        "config.yaml",
    ).read_text(
        encoding="utf-8",
    )
