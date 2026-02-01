from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import yaml

from .types import RoboGymProject


def to_lower_snake_case(name: str) -> str:
    """
    "Start Gate Agent" -> "start_gate_agent"
    "Start-Gate agent" -> "start_gate_agent"
    """
    s = name.strip()
    s = re.sub(r"[^\w\s-]", "", s)
    s = re.sub(r"[-\s]+", "_", s)
    s = re.sub(r"_+", "_", s)
    return s.lower()


def create_project_folder(
    project: RoboGymProject,
    *,
    base_dir: Path | None = None,
) -> Path:
    """
    Creates:
        <base_dir>/projects/<lower_snake_project_name>/config.yaml

    If base_dir is None, uses current working directory.
    Returns the created project directory Path.
    """
    root = base_dir or Path.cwd()

    projects_dir = root / "projects"
    projects_dir.mkdir(parents=True, exist_ok=True)

    folder_name = to_lower_snake_case(project["project_name"])
    project_dir = projects_dir / folder_name

    if project_dir.exists():
        raise FileExistsError(f"Project folder already exists: {project_dir}")

    project_dir.mkdir(parents=True, exist_ok=True)

    config_path = project_dir / "config.yaml"

    cfg: dict[str, Any] = {
        "robogym_project": {
            "name": project["project_name"],
            "world_file": project["world_file"],
            "model_name": project["model_name"],
            "random_spawn_space": {
                "enabled": project["random_spawn_space"]["enabled"],
                # store as yaml list for portability
                "coord_1": list(project["random_spawn_space"]["coord1_4d"]),
                "coord_2": list(project["random_spawn_space"]["coord2_4d"]),
            },
            "input_topics": list(project["input_topics"]),
            "output_topics": list(project["output_topics"]),
        },
    }
    with config_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    return project_dir
