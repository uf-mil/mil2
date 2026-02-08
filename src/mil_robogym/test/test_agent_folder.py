from datetime import datetime
from pathlib import Path

import pytest

from mil_robogym.data_collection.filesystem import (
    create_agent_folder,
    format_agent_timestamp,
)


def test_format_agent_timestamp():
    dt = datetime(2026, 2, 22, 23, 0)  # 11:00 pm
    assert format_agent_timestamp(dt) == "2026_02_22_11_00_pm"


def test_create_agent_folder_creates_files(tmp_path: Path):
    project_dir = tmp_path / "projects" / "start_gate_agent"
    project_dir.mkdir(parents=True)

    # fake weights file
    weights_src = tmp_path / "my_weights.pt"
    weights_src.write_bytes(b"fake-weights")

    metrics = {"reward": [0.1, 0.2, 0.3], "loss": [3.0, 2.0, 1.0]}
    dt = datetime(2026, 2, 22, 23, 0)

    agent_dir = create_agent_folder(
        project_dir,
        trained_model_path=weights_src,
        training_metrics=metrics,
        created_at=dt,
    )

    assert agent_dir.name == "2026_02_22_11_00_pm"
    assert (agent_dir / "weights.pt").read_bytes() == b"fake-weights"
    assert (agent_dir / "training_metrics.csv").exists()


def test_create_agent_folder_raises_if_exists(tmp_path: Path):
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
