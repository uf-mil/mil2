from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any, Mapping, cast

from .filesystem import get_training_project_dir_paths
from .load_saved_agent import load_saved_agent
from .types import RoboGymProjectYaml


def delete_saved_agent(project: Mapping[str, Any], agent_name: str) -> list[Path]:
    """
    Delete a saved agent from every persisted training-artifact directory.

    The primary saved-agent path is validated via ``load_saved_agent`` before any
    delete occurs. Missing mirrored copies are ignored.
    """
    handle = load_saved_agent(project, agent_name)
    project_yaml = cast(RoboGymProjectYaml, {"name": handle.project_name})

    deleted_paths: list[Path] = []
    for project_dir in get_training_project_dir_paths(project_yaml):
        candidate = project_dir / "agents" / handle.agent_name
        if not candidate.exists():
            continue
        shutil.rmtree(candidate)
        deleted_paths.append(candidate)

    return deleted_paths
