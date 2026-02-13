from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)


def count_demo_folders(demos_dir: Path) -> int:
    """
    Count the number of demo subdirectories in a demos directory.

    :param demos_dir: Path to the demos directory.
    :type demos_dir: Path
    :raises ValueError: If 'demos_dir' exists but is not a directory.
    :return: Number of demo subdirectories. Returns '0' if the directory
        does not exist.
    :rtype: int
    """

    if not demos_dir.is_dir():
        raise ValueError("Demo directory path '{}' is not a directory.")

    if not demos_dir.exists():
        return 0

    return sum(
        1
        for entry in demos_dir.iterdir()
        if entry.is_dir() and not entry.name.startswith(".")
    )


def find_projects_dir() -> Path:
    """
    Locate the 'projects' directory within the 'mil_robogym' ROS 2 package.

    :raises RuntimeError: If the 'mil_robogym' package cannot be found (e.g.,
        the code is run outside of a built and sourced ROS 2 workspace).
    :raises FileNotFoundError: If the 'projects' directory does not exist or
        is not a directory.
    :return: Absolute path to the 'projects' directory.
    :rtype: Path
    """
    try:
        share_dir = Path(get_package_share_directory("mil_robogym"))
    except PackageNotFoundError as e:
        raise RuntimeError(
            "Projects directory could not be found because this code was ran without being built into a ROS 2 Package.",
        ) from e

    share_projects = share_dir.joinpath("projects")
    if not share_projects.is_dir():
        raise FileNotFoundError(
            "Projects directory could not be found for unknown reason. Projects directory may not exist.",
        )

    return share_projects


def get_all_project_config() -> list[dict]:
    """
    Load configuration data for all projects in the 'mil_robogym' projects directory.

    :raises RuntimeError: If the projects directory cannot be located.
    :raises FileNotFoundError: If a project directory is missing its
        'config.yaml' file.
    :raises ValueError: If a configuration file cannot be parsed or is empty.
    :return: A list of project configuration dictionaries, each containing
        the parsed YAML data and a 'num_demos' field.
    :rtype: list[dict]
    """
    projects_dir = find_projects_dir()

    configs = []
    for project_dir in projects_dir.iterdir():
        if not project_dir.is_dir() or project_dir.name.startswith("."):
            continue

        config_path = project_dir / "config.yaml"
        if not config_path.exists():
            raise FileNotFoundError(
                f"Config path for project '{project_dir}' does not exist.",
            )

        try:
            parsed = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as e:
            raise ValueError(
                f"Config path for project '{project_dir}' could not be parsed.",
            ) from e

        if not parsed:
            raise ValueError(
                f"Config path for project '{project_dir}' could be parsed, but is empty",
            )

        project_config = dict(parsed)
        project_config["num_demos"] = count_demo_folders(project_dir / "demos")
        configs.append(project_config)

    return configs
