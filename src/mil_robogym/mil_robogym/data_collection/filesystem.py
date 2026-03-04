from __future__ import annotations

import re
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from .types import RoboGymDemo, RoboGymDemoConfig, RoboGymProject


def _resolve_default_base_dir() -> Path:  # TODO: Move to utilsi.py
    """
    Resolve the package share root for mil_robogym:
        <install_prefix>/share/mil_robogym
    """
    try:
        return Path(get_package_share_directory("mil_robogym"))
    except PackageNotFoundError as e:
        raise RuntimeError(
            "mil_robogym package share directory could not be found. "
            "Build and source the ROS 2 workspace first.",
        ) from e


def to_lower_snake_case(name: str) -> str:  # TODO: Move to utils.py
    """
    "Start Gate Agent" -> "start_gate_agent"
    "Start-Gate agent" -> "start_gate_agent"
    """
    s = name.strip()
    s = re.sub(r"[^\w\s-]", "", s)
    s = re.sub(r"[-\s]+", "_", s)
    s = re.sub(r"_+", "_", s)
    return s.lower()


def format_agent_timestamp(dt: datetime) -> str:  # TODO: Make private
    # 12-hour color with am/pm, zero padding
    hour_12 = dt.strftime("%I")
    ampm = dt.strftime("%p").lower()
    return f"{dt:%Y_%m_%d}_{hour_12}_{dt:%M}_{ampm}"


def create_project_folder(
    project: RoboGymProject,
) -> Path:  # TODO: Also save in source directory as well
    """
    Creates:
        <share_dir>/projects/<lower_snake_project_name>/config.yaml

    Uses the ROS 2 package share directory for 'mil_robogym' as the root.
    Returns the created project directory Path.
    """
    root = _resolve_default_base_dir()

    projects_dir = root / "projects"
    projects_dir.mkdir(parents=True, exist_ok=True)

    folder_name = to_lower_snake_case(project["project_name"])
    project_dir = projects_dir / folder_name

    if project_dir.exists():
        raise FileExistsError(f"Project folder already exists: {project_dir}")

    project_dir.mkdir(parents=True, exist_ok=True)

    demo_dir = project_dir / "demos"
    demo_dir.mkdir(exist_ok=True)

    config_path = project_dir / "config.yaml"

    cfg: dict[str, Any] = {  # TODO: Make a type for this
        "robogym_project": {
            "name": project["project_name"],
            "world_file": project["world_file"],
            "model_name": project["model_name"],
            "random_spawn_space": {
                "enabled": project["random_spawn_space"]["enabled"],
                # store as yaml list for portability
                "coord1_4d": list(project["random_spawn_space"]["coord1_4d"]),
                "coord2_4d": list(project["random_spawn_space"]["coord2_4d"]),
            },
            "input_topics": list(project["input_topics"]),
            "output_topics": list(project["output_topics"]),
        },
    }

    tensor_spec = project.get("tensor_spec")
    if tensor_spec is not None:
        cfg["robogym_project"]["tensor_spec"] = {
            "input_features": list(tensor_spec["input_features"]),
            "output_features": list(tensor_spec["output_features"]),
            "input_dim": int(tensor_spec["input_dim"]),
            "output_dim": int(tensor_spec["output_dim"]),
            "ignored_input_features": {
                topic: list(fields)
                for topic, fields in tensor_spec["ignored_input_features"].items()
            },
            "ignored_output_features": {
                topic: list(fields)
                for topic, fields in tensor_spec["ignored_output_features"].items()
            },
        }

    with config_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    return project_dir


def edit_project(
    project: RoboGymProject,
    *,
    original_project_name: str | None = None,
) -> Path:  # TODO: Apply changes to source as well
    """
    Edit an existing project's config.yaml using a RoboGymProject payload.

    Writes:
        <share_dir>/projects/<lower_snake_project_name>/config.yaml

    Returns the existing project directory Path.
    """
    root = _resolve_default_base_dir()
    projects_dir = root / "projects"
    current_name = original_project_name or project["project_name"]
    current_folder_name = to_lower_snake_case(current_name)
    project_dir = projects_dir / current_folder_name

    if not project_dir.exists() or not project_dir.is_dir():
        raise FileNotFoundError(f"Project folder does not exist: {project_dir}")

    target_folder_name = to_lower_snake_case(project["project_name"])
    target_project_dir = projects_dir / target_folder_name
    if target_project_dir != project_dir:
        if target_project_dir.exists():
            raise FileExistsError(
                f"Cannot rename project; target folder already exists: {target_project_dir}",
            )
        project_dir.rename(target_project_dir)
        project_dir = target_project_dir

    config_path = project_dir / "config.yaml"
    cfg: dict[str, Any] = {  # TODO: Apply typing
        "robogym_project": {
            "name": project["project_name"],
            "world_file": project["world_file"],
            "model_name": project["model_name"],
            "random_spawn_space": {
                "enabled": project["random_spawn_space"]["enabled"],
                # store as yaml list for portability
                "coord1_4d": list(project["random_spawn_space"]["coord1_4d"]),
                "coord2_4d": list(project["random_spawn_space"]["coord2_4d"]),
            },
            "input_topics": list(project["input_topics"]),
            "output_topics": list(project["output_topics"]),
        },
    }

    tensor_spec = project.get("tensor_spec")
    if tensor_spec is not None:
        cfg["robogym_project"]["tensor_spec"] = {
            "input_features": list(tensor_spec["input_features"]),
            "output_features": list(tensor_spec["output_features"]),
            "input_dim": int(tensor_spec["input_dim"]),
            "output_dim": int(tensor_spec["output_dim"]),
            "ignored_input_features": {
                topic: list(fields)
                for topic, fields in tensor_spec["ignored_input_features"].items()
            },
            "ignored_output_features": {
                topic: list(fields)
                for topic, fields in tensor_spec["ignored_output_features"].items()
            },
        }
    with config_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    return project_dir


def edit_demo(
    project: RoboGymProject,
    demo: RoboGymDemo,
    *,
    original_demo_name: str | None = None,
) -> Path:
    """
    Edit an existing demo's config.yaml using a RoboGymDemo payload.

    Writes:
        <share_dir>/projects/<lower_snake_project_name>/demos/<lower_snake_demo_name>/config.yaml

    Returns the existing demo directory Path
    """
    root = _resolve_default_base_dir()
    projects_dir = root / "projects"
    project_name = to_lower_snake_case(project["name"])
    demo_name = to_lower_snake_case(original_demo_name or demo["demo_name"])
    demo_dir = projects_dir / project_name / "demos" / demo_name

    if not demo_dir.exists() or not demo_dir.is_dir():
        raise FileNotFoundError(f"Demo folder does not exist: {demo_dir}")

    target_folder_name = to_lower_snake_case(demo["demo_name"])
    target_demo_dir = demo_dir.parent / target_folder_name
    if target_demo_dir != demo_dir:
        if target_demo_dir.exists():
            raise FileExistsError(
                f"Cannot rename demo; target folder already exists: {target_demo_dir}",
            )
        demo_dir.rename(target_demo_dir)
        demo_dir = target_demo_dir

    config_path = demo_dir / "config.yaml"
    cfg: RoboGymDemoConfig = {
        "robogym_demo": {
            "demo_name": demo["demo_name"],
            "start_position": demo["start_position"],
            "sampling_rate": demo["sampling_rate"],
        },
    }

    with config_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    return demo_dir


def create_agent_folder(
    project_dir: Path,
    *,
    trained_model_path: Path,
    training_metrics: dict[str, list[float]],
    created_at: datetime | None,
) -> Path:
    """
    Create a timestamped agent folder under <project_dir>/agents/

    Writes:
      - weights.pt (copied from trained_model_path)
      - training_metrics.csv
      - metrics/*.png (one plot per metric series)

    Raises:
      FileExistsError if the timestamp folder already exists
      FileNotFoundError if trained_model_path does not exist
      ValueError if training_metrics is empty or lengths mismatch
    """
    if not trained_model_path.exists():
        raise FileNotFoundError(f"trained_model_path not found: {trained_model_path}")

    if not training_metrics:
        raise ValueError("training_metrics is empty")

    # Validate metric lengths are consistent
    lengths = {len(v) for v in training_metrics.values()}
    if len(lengths) != 1:
        raise ValueError(f"training_metrics series lengths mismatch: {sorted(lengths)}")
    n = lengths.pop()

    dt = created_at or datetime.now()
    agent_name = format_agent_timestamp(dt)

    agents_dir = project_dir / "agents"
    agents_dir.mkdir(parents=True, exist_ok=True)

    agent_dir = agents_dir / agent_name
    if agent_dir.exists():
        raise FileExistsError(f"Agent folder already exists: {agent_dir}")
    agent_dir.mkdir(parents=True, exist_ok=False)

    # 1) weights.pt
    shutil.copyfile(trained_model_path, agent_dir / "weights.pt")

    # 2) metrics CSV
    csv_path = agent_dir / "training_metrics.csv"
    headers = ["index", *training_metrics.keys()]
    with csv_path.open("w", encoding="utf-8") as f:
        f.write(",".join(headers) + "\n")
        for i in range(n):
            row = [str(i)] + [str(training_metrics[k][i]) for k in training_metrics]
            f.write(",".join(row) + "\n")

    # 3) plots
    # (Optional but matches issue: "images of graphs showing the metrics")
    metrics_dir = agent_dir / "metrics"
    metrics_dir.mkdir(parents=True, exist_ok=True)

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        xs = list(range(n))
        for name, series in training_metrics.items():
            plt.figure()
            plt.plot(xs, series)
            plt.title(name)
            plt.xlabel("index")
            plt.ylabel(name)
            plt.tight_layout()
            plt.savefig(metrics_dir / f"{name}.png")
            plt.close()
    except Exception as e:
        raise RuntimeError(
            "Failed to generate training metric plots. "
            "Ensure matplotlib is installed and metrics are valid.",
        ) from e

    return agent_dir


def create_demo_folder(
    project_dir: Path,
    *,
    demo_name: str,
    sampling_rate: float,
    start_position: (
        tuple[float, float, float, float] | None
    ) = None,  # TODO: Create type for start position
) -> (Path, dict[str, Any]):
    """
    Create a demo folder under <project_dir>/demos/ with a config.yaml.

    Creates:
        <project_dir>/demos/<lower_snake_demo_name>/config.yaml

    If start_position is None, defaults to (0.0, 0.0, 0.0, 0.0).
    Returns the created demo directory Path.
    """
    demos_dir = project_dir / "demos"
    demos_dir.mkdir(parents=True, exist_ok=True)

    folder_name = to_lower_snake_case(demo_name)
    demo_dir = demos_dir / folder_name

    if demo_dir.exists():
        raise FileExistsError(f"Demo folder already exists: {demo_dir}")

    demo_dir.mkdir(parents=True, exist_ok=False)

    if start_position is None:
        start_position = (0.0, 0.0, 0.0, 0.0)

    config_path = demo_dir / "config.yaml"
    cfg: dict[str, Any] = {  # TODO: Apply typing for this
        "robogym_demo": {
            "demo_name": demo_name,
            "start_position": list(start_position),
            "sampling_rate": sampling_rate,
        },
    }
    with config_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    return demo_dir, cfg
