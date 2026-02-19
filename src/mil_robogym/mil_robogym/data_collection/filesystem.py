from __future__ import annotations

import re
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from .types import RoboGymProject


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


def format_agent_timestamp(dt: datetime) -> str:
    # 12-hour color with am/pm, zero padding
    hour_12 = dt.strftime("%I")
    ampm = dt.strftime("%p").lower()
    return f"{dt:%Y_%m_%d}_{hour_12}_{dt:%M}_{ampm}"


def create_project_folder(
    project: RoboGymProject,
    *,
    base_dir: Path | None = None,
) -> Path:
    """
    Creates:
        <base_dir>/projects/<lower_snake_project_name>/config.yaml

    If base_dir is None, uses current working directory.
    Returns the created project directory Path.
    """
    root = base_dir or Path.cwd()

    projects_dir = root / "projects"
    projects_dir.mkdir(parents=True, exist_ok=True)

    folder_name = to_lower_snake_case(project["project_name"])
    project_dir = projects_dir / folder_name

    if project_dir.exists():
        raise FileExistsError(f"Project folder already exists: {project_dir}")

    project_dir.mkdir(parents=True, exist_ok=True)

    config_path = project_dir / "config.yaml"

    cfg: dict[str, Any] = {
        "robogym_project": {
            "name": project["project_name"],
            "world_file": project["world_file"],
            "model_name": project["model_name"],
            "random_spawn_space": {
                "enabled": project["random_spawn_space"]["enabled"],
                # store as yaml list for portability
                "coord_1": list(project["random_spawn_space"]["coord1_4d"]),
                "coord_2": list(project["random_spawn_space"]["coord2_4d"]),
            },
            "input_topics": list(project["input_topics"]),
            "output_topics": list(project["output_topics"]),
        },
    }
    with config_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    return project_dir


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
    start_position: tuple[float, float, float, float] | None = None,
) -> Path:
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
    cfg: dict[str, Any] = {
        "robogym_demo": {
            "demo_name": demo_name,
            "start_position": list(start_position),
            "sampling_rate": sampling_rate,
        },
    }
    with config_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    return demo_dir
