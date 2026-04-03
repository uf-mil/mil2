from __future__ import annotations

import shutil
from typing import Any, Mapping, cast

from .filesystem import get_demo_dir_path
from .types import RoboGymDemoYaml, RoboGymProjectYaml


def delete_demo(project: Mapping[str, Any], demo: Mapping[str, Any]) -> None:
    """
    Delete a demo directory for the given project and demo payloads.

    Accepts either full config mappings (with ``robogym_project`` /
    ``robogym_demo`` keys) or raw yaml mappings directly.
    """
    project_yaml_raw = project.get("robogym_project", project)
    demo_yaml_raw = demo.get("robogym_demo", demo)

    if not isinstance(project_yaml_raw, Mapping):
        raise ValueError("Project payload must be a mapping.")
    if not isinstance(demo_yaml_raw, Mapping):
        raise ValueError("Demo payload must be a mapping.")

    project_name = project_yaml_raw.get("name")
    demo_name = demo_yaml_raw.get("name")

    if not isinstance(project_name, str) or not project_name.strip():
        raise ValueError("Project payload must include a non-empty 'name'.")
    if not isinstance(demo_name, str) or not demo_name.strip():
        raise ValueError("Demo payload must include a non-empty 'name'.")

    project_yaml = cast(RoboGymProjectYaml, {"name": project_name})
    demo_yaml = cast(RoboGymDemoYaml, {"name": demo_name})

    demo_dir = get_demo_dir_path(project_yaml, demo_yaml)
    if demo_dir.exists():
        shutil.rmtree(demo_dir)
