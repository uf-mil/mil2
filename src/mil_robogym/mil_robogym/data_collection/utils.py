from __future__ import annotations

import os
import re
from pathlib import Path

SOURCE_PROJECTS_DIR_ENV = "MIL_ROBOGYM_SOURCE_PROJECTS_DIR"


def resolve_package_share_dir(package_name: str = "mil_robogym") -> Path:
    """
    Resolve a package share directory from the ROS 2 ament index.
    """
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ModuleNotFoundError as e:
        raise RuntimeError(
            "ament_index_python is required to resolve ROS 2 package share directories.",
        ) from e

    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError as e:
        raise RuntimeError(
            f"{package_name} package share directory could not be found. "
            "Build and source the ROS 2 workspace first.",
        ) from e


def resolve_source_projects_dir(
    share_dir: Path,
    *,
    package_name: str = "mil_robogym",
) -> Path | None:
    """
    Resolve the workspace source projects directory:
      <workspace_root>/src/<package_name>/projects

    Returns None when a source workspace cannot be inferred.
    """
    override = os.environ.get(SOURCE_PROJECTS_DIR_ENV)
    if override:
        return Path(override).expanduser().resolve()

    resolved_share_dir = share_dir.resolve()
    install_dir = next(
        (
            parent
            for parent in (resolved_share_dir, *resolved_share_dir.parents)
            if parent.name == "install"
        ),
        None,
    )
    if install_dir is None:
        return None

    workspace_root = install_dir.parent
    package_dir = workspace_root / "src" / package_name
    if not package_dir.is_dir():
        return None

    return package_dir / "projects"


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


def canonical_topic_name(topic: str) -> str:
    """Normalize a topic name by trimming whitespace and leading slashes."""
    return topic.strip().lstrip("/")


def topic_to_data_folder_name(topic: str) -> str:
    """
    Convert a topic name into a filesystem-safe folder name.

    Example:
        "/frontcam/image_raw" -> "frontcam_image_raw"
    """
    normalized = canonical_topic_name(topic)
    normalized = normalized.replace("/", "_")
    normalized = re.sub(r"[^\w.-]", "_", normalized)
    normalized = re.sub(r"_+", "_", normalized).strip("_")
    return normalized or "topic"
