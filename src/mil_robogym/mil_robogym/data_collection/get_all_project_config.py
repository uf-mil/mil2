from __future__ import annotations

from pathlib import Path

import yaml

from .utils import resolve_package_share_dir


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

    if not demos_dir.exists():
        return 0

    if not demos_dir.is_dir():
        raise ValueError(f"Demo directory path '{demos_dir}' is not a directory.")

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
    :raises FileNotFoundError: If the 'projects' path exists but is not a
        directory.
    :return: Absolute path to the 'projects' directory.
    :rtype: Path
    """
    try:
        share_dir = resolve_package_share_dir("mil_robogym")
    except RuntimeError as e:
        raise RuntimeError(
            "Projects directory could not be found because this code was ran without being built into a ROS 2 Package.",
        ) from e

    share_projects = share_dir.joinpath("projects")
    if not share_projects.exists():
        share_projects.mkdir(parents=True, exist_ok=True)
    elif not share_projects.is_dir():
        raise FileNotFoundError(
            "Projects path exists but is not a directory.",
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

        if not isinstance(parsed, dict):
            raise ValueError(
                f"Config path for project '{project_dir}' is not a mapping.",
            )

        try:
            robogym_project = parsed["robogym_project"]
            project_name = robogym_project["name"]
            input_topics = robogym_project["input_topics"]
            output_topics = robogym_project["output_topics"]
        except (KeyError, TypeError) as e:
            raise ValueError(
                "Project config must include "
                "'robogym_project.name' and "
                "'robogym_project.{input_topics,output_topics}'.",
            ) from e

        if not isinstance(project_name, str) or not project_name.strip():
            raise ValueError(
                f"Project config for '{project_dir}' has an invalid name.",
            )

        if not isinstance(input_topics, dict) or not isinstance(output_topics, dict):
            raise ValueError(
                f"Project config for '{project_dir}' has invalid topic mappings.",
            )

        for list_name, topic_map in (
            ("input_topics", input_topics),
            ("output_topics", output_topics),
        ):
            for topic, subtopics in topic_map.items():
                if not isinstance(topic, str):
                    raise ValueError(
                        f"Project config for '{project_dir}' has a non-string topic in {list_name}.",
                    )
                if not isinstance(subtopics, list):
                    raise ValueError(
                        f"Project config for '{project_dir}' has non-list subtopics in {list_name}.",
                    )

        project_config = dict(parsed)
        project_config["num_demos"] = count_demo_folders(project_dir / "demos")
        configs.append(project_config)

    return configs
