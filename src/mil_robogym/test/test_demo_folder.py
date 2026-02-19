from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.filesystem import create_demo_folder


def test_create_demo_folder_creates_files(tmp_path: Path):
    project_dir = tmp_path / "projects" / "start_gate_agent"
    project_dir.mkdir(parents=True)

    demo_dir = create_demo_folder(
        project_dir,
        demo_name="Demo 1",
        sampling_rate=1.0,
        start_position=(1.0, 2.0, 3.0, 4.0),
    )

    assert demo_dir.exists()
    assert demo_dir.name == "demo_1"

    cfg = yaml.safe_load((demo_dir / "config.yaml").read_text(encoding="utf-8"))
    assert cfg["robogym_demo"]["demo_name"] == "Demo 1"
    assert cfg["robogym_demo"]["sampling_rate"] == 1.0
    assert cfg["robogym_demo"]["start_position"] == [1.0, 2.0, 3.0, 4.0]


def test_create_demo_folder_raises_if_exists(tmp_path: Path):
    project_dir = tmp_path / "projects" / "demo_project"
    project_dir.mkdir(parents=True)

    create_demo_folder(
        project_dir,
        demo_name="Demo 1",
        sampling_rate=1.0,
    )

    with pytest.raises(FileExistsError):
        create_demo_folder(
            project_dir,
            demo_name="Demo 1",
            sampling_rate=1.0,
        )
