from __future__ import annotations

import yaml

from .filesystem import to_lower_snake_case
from .get_all_project_config import find_projects_dir


def get_all_demo_config(project_name: str) -> dict[str, dict]:  # TODO: TYPING!
    """
    Load all demo config YAMLs for a project and return:
    demo_name_raw -> parsed_yaml_dict

    The project folder path is resolved from ``project_name`` converted to
    lower snake case and looked up under:
        <projects_dir>/<lower_snake_project_name>/demos
    """
    projects_dir = (
        find_projects_dir()
    )  # TODO: We know where the folder is just one check is enough.
    project_dir = projects_dir / to_lower_snake_case(project_name)

    if not project_dir.is_dir():
        raise FileNotFoundError(
            f"Project directory does not exist for project '{project_name}': {project_dir}",
        )

    demos_dir = project_dir / "demos"
    if not demos_dir.is_dir():
        raise FileNotFoundError(
            f"Demos directory does not exist for project '{project_name}': {demos_dir}",
        )

    configs: dict[str, dict] = {}  # TODO: TYPING
    for demo_dir in demos_dir.iterdir():
        if not demo_dir.is_dir() or demo_dir.name.startswith("."):
            continue

        config_path = demo_dir / "config.yaml"
        if not config_path.is_file():
            raise FileNotFoundError(
                f"Config path for demo '{demo_dir.name}' does not exist: {config_path}",
            )

        try:
            parsed = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as e:
            raise ValueError(
                f"Config path for demo '{demo_dir.name}' could not be parsed: {config_path}",
            ) from e

        if not isinstance(parsed, dict) or not parsed:
            raise ValueError(
                f"Config path for demo '{demo_dir.name}' parsed, but is empty or invalid: {config_path}",
            )

        # Store by the raw name from YAML (not snake_case folder name).
        demo_name_raw = parsed.get("robogym_demo", {}).get(
            "demo_name",
        )  # TODO: If structure is invalid propagate exception
        if not isinstance(demo_name_raw, str) or not demo_name_raw.strip():
            raise ValueError(
                f"Config for demo '{demo_dir.name}' is missing robogym_demo.demo_name: {config_path}",
            )

        configs[demo_name_raw] = parsed

    return configs
