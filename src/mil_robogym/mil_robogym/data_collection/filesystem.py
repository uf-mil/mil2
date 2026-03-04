from __future__ import annotations

import shutil
from datetime import datetime
from pathlib import Path

import yaml

from .types import (
    Coord4D,
    RoboGymDemoConfig,
    RoboGymDemoYaml,
    RoboGymProject,
    RoboGymProjectConfig,
    RoboGymProjectYaml,
)
from .utils import (
    resolve_package_share_dir,
    resolve_source_projects_dir,
    to_lower_snake_case,
)


def _format_agent_timestamp(dt: datetime) -> str:
    # 12-hour color with am/pm, zero padding
    hour_12 = dt.strftime("%I")
    ampm = dt.strftime("%p").lower()
    return f"{dt:%Y_%m_%d}_{hour_12}_{dt:%M}_{ampm}"


def _project_roots() -> list[Path]:
    share_dir = resolve_package_share_dir()
    share_projects = share_dir / "projects"
    source_projects = resolve_source_projects_dir(share_dir)

    roots = [share_projects]
    if (
        source_projects is not None
        and source_projects.resolve() != share_projects.resolve()
    ):
        roots.append(source_projects)
    return roots


def _build_project_config(project: RoboGymProject) -> RoboGymProjectConfig:
    cfg_project: RoboGymProjectYaml = {
        "name": project["project_name"],
        "world_file": project["world_file"],
        "model_name": project["model_name"],
        "random_spawn_space": {
            "enabled": project["random_spawn_space"]["enabled"],
            # Store as yaml lists for portability.
            "coord1_4d": list(project["random_spawn_space"]["coord1_4d"]),
            "coord2_4d": list(project["random_spawn_space"]["coord2_4d"]),
        },
        "input_topics": list(project["input_topics"]),
        "output_topics": list(project["output_topics"]),
    }

    tensor_spec = project.get("tensor_spec")
    if tensor_spec is not None:
        cfg_project["tensor_spec"] = {
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
    return {"robogym_project": cfg_project}


def _build_demo_config(
    *,
    demo_name: str,
    sampling_rate: float,
    start_position: Coord4D,
) -> RoboGymDemoConfig:
    cfg_demo: RoboGymDemoYaml = {
        "demo_name": demo_name,
        "start_position": list(start_position),
        "sampling_rate": sampling_rate,
    }
    return {"robogym_demo": cfg_demo}


def _write_yaml_config(
    config_path: Path,
    cfg: RoboGymProjectConfig | RoboGymDemoConfig,
) -> None:
    with config_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)


def create_project_folder(
    project: RoboGymProject,
) -> Path:
    """
    Creates:
        <share_dir>/projects/<lower_snake_project_name>/config.yaml

    Uses the ROS 2 package share directory for 'mil_robogym' as the root.
    Returns the created project directory Path.
    """
    folder_name = to_lower_snake_case(project["project_name"])
    project_roots = _project_roots()
    project_dirs = [root / folder_name for root in project_roots]

    for root in project_roots:
        root.mkdir(parents=True, exist_ok=True)

    existing = next(
        (project_dir for project_dir in project_dirs if project_dir.exists()),
        None,
    )
    if existing is not None:
        raise FileExistsError(f"Project folder already exists: {existing}")

    for project_dir in project_dirs:
        project_dir.mkdir(parents=True, exist_ok=False)
        (project_dir / "demos").mkdir(exist_ok=True)

    cfg = _build_project_config(project)
    for project_dir in project_dirs:
        _write_yaml_config(project_dir / "config.yaml", cfg)

    return project_dirs[0]


def edit_project(
    project: RoboGymProject,
    *,
    original_project_name: str | None = None,
) -> Path:
    """
    Edit an existing project's config.yaml using a RoboGymProject payload.

    Writes:
        <share_dir>/projects/<lower_snake_project_name>/config.yaml

    Returns the existing project directory Path.
    """
    project_roots = _project_roots()
    share_projects_dir = project_roots[0]
    source_projects_dir = project_roots[1] if len(project_roots) > 1 else None

    current_name = original_project_name or project["project_name"]
    current_folder_name = to_lower_snake_case(current_name)
    project_dir = share_projects_dir / current_folder_name

    if not project_dir.exists() or not project_dir.is_dir():
        raise FileNotFoundError(f"Project folder does not exist: {project_dir}")

    target_folder_name = to_lower_snake_case(project["project_name"])
    target_project_dir = share_projects_dir / target_folder_name

    source_project_dir: Path | None = None
    source_target_project_dir: Path | None = None
    if source_projects_dir is not None:
        source_projects_dir.mkdir(parents=True, exist_ok=True)
        source_project_dir = source_projects_dir / current_folder_name
        source_target_project_dir = source_projects_dir / target_folder_name
        if (
            source_project_dir.exists()
            and source_target_project_dir != source_project_dir
            and source_target_project_dir.exists()
        ):
            raise FileExistsError(
                "Cannot rename project in source tree; target folder already exists: "
                f"{source_target_project_dir}",
            )

    if target_project_dir != project_dir:
        if target_project_dir.exists():
            raise FileExistsError(
                f"Cannot rename project; target folder already exists: {target_project_dir}",
            )
        project_dir.rename(target_project_dir)
        project_dir = target_project_dir

    if source_projects_dir is not None and source_target_project_dir is not None:
        if source_project_dir is not None and source_project_dir.exists():
            if source_target_project_dir != source_project_dir:
                source_project_dir.rename(source_target_project_dir)
        else:
            source_target_project_dir.mkdir(parents=True, exist_ok=True)
        (source_target_project_dir / "demos").mkdir(exist_ok=True)

    cfg = _build_project_config(project)
    _write_yaml_config(project_dir / "config.yaml", cfg)
    if source_target_project_dir is not None:
        _write_yaml_config(source_target_project_dir / "config.yaml", cfg)

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
    agent_name = _format_agent_timestamp(dt)

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
    start_position: Coord4D | None = None,
) -> tuple[Path, RoboGymDemoConfig]:
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
    cfg = _build_demo_config(
        demo_name=demo_name,
        sampling_rate=sampling_rate,
        start_position=start_position,
    )
    _write_yaml_config(config_path, cfg)

    return demo_dir, cfg
