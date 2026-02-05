from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def count_demo_folders(demos_dir) -> int:
    if not demos_dir.exists() or not demos_dir.is_dir():
        return 0

    return sum(
        1
        for entry in demos_dir.iterdir()
        if entry.is_dir() and not entry.name.startswith(".")
    )


def find_projects_dir() -> Path:
    try:
        share_dir = Path(get_package_share_directory("mil_robogym"))
    except Exception:
        raise Exception(
            "Projects directory could not be found because this code was ran without being built into a ROS 2 Package.",
        )

    share_projects = share_dir.joinpath("projects")
    if not share_projects.is_dir():
        raise Exception(
            "Projects directory could not be found for unknown reason. Projects directory may not exist.",
        )

    return share_projects


def get_all_project_config() -> list[dict]:
    projects_dir = find_projects_dir()

    configs = []
    for project_dir in projects_dir.iterdir():
        if not project_dir.is_dir() or project_dir.name.startswith("."):
            continue

        config_path = project_dir / "config.yaml"
        if not config_path.exists():
            raise Exception(f"Config path for project '{project_dir}' does not exist.")

        try:
            parsed = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError):
            raise Exception(
                f"Config path for project '{project_dir}' could not be parsed.",
            )

        if not parsed:
            raise Exception(
                f"Config path for project '{project_dir}' could be parsed, but is empty",
            )

        project_config = dict(parsed)
        project_config["num_demos"] = count_demo_folders(project_dir / "demos")
        configs.append(project_config)

    return configs
