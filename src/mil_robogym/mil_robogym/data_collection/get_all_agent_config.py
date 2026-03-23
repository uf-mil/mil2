from __future__ import annotations

from pathlib import Path

import yaml


def get_num_demos(agent_dir: Path) -> int:
    """Read an agent config and return the number of demos it uses."""
    config_path = agent_dir / "config.yaml"
    if not config_path.is_file():
        raise FileNotFoundError(
            f"Config path for agent '{agent_dir.name}' does not exist: {config_path}",
        )

    try:
        parsed = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as e:
        raise ValueError(
            f"Config path for agent '{agent_dir.name}' could not be parsed: {config_path}",
        ) from e

    if not isinstance(parsed, dict) or not parsed:
        raise ValueError(
            f"Config path for agent '{agent_dir.name}' parsed, but is empty or invalid: {config_path}",
        )

    try:
        num_demos = parsed["robogym_agent"]["num_demos"]
    except (KeyError, TypeError) as e:
        raise ValueError(
            f"Config for agent '{agent_dir.name}' has an invalid robogym_agent structure: {config_path}",
        ) from e

    try:
        return int(num_demos)
    except (TypeError, ValueError) as e:
        raise ValueError(
            f"Config for agent '{agent_dir.name}' has an invalid num_demos value: {config_path}",
        ) from e


def get_all_agent_config(project_dir: Path) -> dict[str, dict[str, int]]:
    """Load all agents for a project as {agent_name: {"num_demos": int}}."""
    if not project_dir.is_dir():
        raise FileNotFoundError(f"Project directory does not exist: {project_dir}")

    agents_dir = project_dir / "agents"
    if not agents_dir.is_dir():
        agents_dir.mkdir(parents=True, exist_ok=True)
        return {}

    configs: dict[str, dict[str, int]] = {}
    for agent_dir in agents_dir.iterdir():
        if not agent_dir.is_dir() or agent_dir.name.startswith("."):
            continue

        configs[agent_dir.name] = {
            "num_demos": get_num_demos(agent_dir),
        }

    return configs
