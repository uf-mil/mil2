"""Tests for editing demo configs with the name-based project schema."""

from pathlib import Path

import yaml

from mil_robogym.data_collection.filesystem import edit_demo


def test_edit_demo_uses_name_schema(tmp_path: Path, monkeypatch):
    """Updates demo config using project['name'] for project lookup."""
    share_dir = tmp_path / "install" / "mil_robogym" / "share" / "mil_robogym"
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_package_share_dir",
        lambda: share_dir,
    )
    monkeypatch.setattr(
        "mil_robogym.data_collection.filesystem.resolve_source_projects_dir",
        lambda _share_dir: None,
    )

    demo_dir = share_dir / "projects" / "start_gate_agent" / "demos" / "demo_1"
    demo_dir.mkdir(parents=True)
    (demo_dir / "config.yaml").write_text(
        yaml.safe_dump(
            {
                "robogym_demo": {
                    "name": "Demo 1",
                    "start_position": [0.0, 0.0, 0.0, 0.0],
                    "sampling_rate": 1.0,
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    updated_dir = edit_demo(
        {"name": "Start Gate Agent"},
        {
            "name": "Demo 1",
            "start_position": [1.0, 2.0, 3.0, 4.0],
            "sampling_rate": 5.0,
        },
    )

    assert updated_dir == demo_dir
    cfg = yaml.safe_load((demo_dir / "config.yaml").read_text(encoding="utf-8"))
    assert cfg["robogym_demo"]["start_position"] == [1.0, 2.0, 3.0, 4.0]
    assert cfg["robogym_demo"]["sampling_rate"] == 5.0
