from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.filesystem import (
    create_project_folder,
    to_lower_snake_case,
)


def test_create_project_folder(tmp_path: Path, monkeypatch):
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.get_package_share_directory",
        lambda _pkg: str(tmp_path),
    )

    proj = {
        "project_name": "Start Gate Agent",
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": (0.0, 0.0, 0.0, 0.0),
            "coord2_4d": (1.0, 2.0, 3.0, 4.0),
        },
        "input_topics": ["imu/processed", "dvl/processed"],
        "output_topics": ["trajectory/4_deg"],
    }

    project_dir = create_project_folder(proj)

    assert project_dir.exists()
    assert project_dir.name == "start_gate_agent"

    cfg = (project_dir / "config.yaml").read_text()
    assert "robogym_project" in cfg
    assert "Start Gate Agent" in cfg
    assert "imu/processed" in cfg


def test_to_lower_snake_case_basic():
    assert to_lower_snake_case("Start Gate Agent") == "start_gate_agent"


def test_to_lower_snake_case_hyphens_and_spaces():
    assert to_lower_snake_case("Start-Gate agent") == "start_gate_agent"
    assert to_lower_snake_case("Start   Gate---Agent") == "start_gate_agent"


def test_to_lower_snake_case_punctuation():
    assert to_lower_snake_case("Start Gate: Agent!") == "start_gate_agent"
    assert to_lower_snake_case("  Start (Gate) Agent  ") == "start_gate_agent"


def test_to_lower_snake_case_already_snake():
    assert to_lower_snake_case("start_gate_agent") == "start_gate_agent"


def test_create_project_folder_raises_if_exists(tmp_path, monkeypatch):
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.get_package_share_directory",
        lambda _pkg: str(tmp_path),
    )

    proj = {
        "project_name": "Start Gate Agent",
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": (0.0, 0.0, 0.0, 0.0),
            "coord2_4d": (0.0, 0.0, 0.0, 0.0),
        },
        "input_topics": ["imu/processed", "dvl/processed"],
        "output_topics": ["trajectory/4_deg"],
    }

    # Create the folder first time
    first = create_project_folder(proj)
    assert first.exists()

    # Second time should throw
    with pytest.raises(FileExistsError):
        create_project_folder(proj)


def test_create_project_folder_writes_tensor_spec(tmp_path: Path, monkeypatch):
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.get_package_share_directory",
        lambda _pkg: str(tmp_path),
    )

    proj = {
        "project_name": "Tensor Spec Project",
        "world_file": "src/default/world/file",
        "model_name": "weights.pt",
        "random_spawn_space": {
            "enabled": False,
            "coord1_4d": (0.0, 0.0, 0.0, 0.0),
            "coord2_4d": (1.0, 2.0, 3.0, 4.0),
        },
        "input_topics": ["imu/processed"],
        "output_topics": ["trajectory/4_deg"],
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
