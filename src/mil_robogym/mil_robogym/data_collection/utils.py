from __future__ import annotations

import os
import re
from collections.abc import Sequence
from pathlib import Path

SOURCE_PROJECTS_DIR_ENV = "MIL_ROBOGYM_SOURCE_PROJECTS_DIR"
SOURCE_PACKAGE_DIR_ENV = "MIL_ROBOGYM_SOURCE_PACKAGE_DIR"


def flatten_value(value: object, prefix: str, out: dict[str, object]) -> None:
    """
    Flatten nested dictionaries/lists into dot and bracket notation keys.

    Examples:
        pose.position.x
        covariance[0]
    """
    if isinstance(value, dict):
        if not value and prefix:
            out[prefix] = {}
            return
        for key in sorted(value, key=str):
            key_str = str(key)
            child_prefix = f"{prefix}.{key_str}" if prefix else key_str
            flatten_value(value[key], child_prefix, out)
        return

    if isinstance(value, list):
        if not value and prefix:
            out[prefix] = []
            return
        for index, item in enumerate(value):
            child_prefix = f"{prefix}[{index}]" if prefix else f"[{index}]"
            flatten_value(item, child_prefix, out)
        return

    if prefix:
        out[prefix] = value
    else:
        out["value"] = value


def extract_selected_state_features(
    state: dict[str, object],
    feature_names: Sequence[str],
) -> dict[str, object]:
    """
    Flatten collected topic messages and return only the requested features.
    """
    flattened_states: dict[str, object] = {}

    for topic, msg in state.items():
        temp: dict[str, object] = {}
        flatten_value(msg, "", temp)
        for key, value in temp.items():
            flattened_states[f"{topic}:{key}"] = value

    return {
        feature_name: flattened_states[feature_name]
        for feature_name in feature_names
        if feature_name in flattened_states
    }


def resolve_source_package_dir(package_name: str = "mil_robogym") -> Path:
    """
    Resolve the package source directory:
      <workspace_root>/src/<package_name>

    Resolution order:
      1) MIL_ROBOGYM_SOURCE_PACKAGE_DIR override
      2) Walk from this file location (supports running from source tree)
      3) Infer from install tree (supports running installed code in same workspace)
    """
    override = os.environ.get(SOURCE_PACKAGE_DIR_ENV)
    if override:
        return Path(override).expanduser().resolve()

    this_file = Path(__file__).resolve()
    for parent in (this_file, *this_file.parents):
        if parent.name == package_name and parent.parent.name == "src":
            return parent

    install_dir = next(
        (
            parent
            for parent in (this_file, *this_file.parents)
            if parent.name == "install"
        ),
        None,
    )
    if install_dir is not None:
        workspace_root = install_dir.parent
        candidate = workspace_root / "src" / package_name
        if candidate.is_dir():
            return candidate

    raise RuntimeError(
        f"Could not resolve source directory for package '{package_name}'. "
        f"Set {SOURCE_PACKAGE_DIR_ENV} or {SOURCE_PROJECTS_DIR_ENV}.",
    )


def resolve_source_projects_dir(
    *,
    package_name: str = "mil_robogym",
) -> Path:
    """
    Resolve the workspace source projects directory:
      <workspace_root>/src/<package_name>/projects
    """
    override = os.environ.get(SOURCE_PROJECTS_DIR_ENV)
    if override:
        return Path(override).expanduser().resolve()

    package_dir = resolve_source_package_dir(package_name=package_name)
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
