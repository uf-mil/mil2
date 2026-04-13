from __future__ import annotations

import csv
import shutil
from collections.abc import Mapping
from datetime import datetime
from pathlib import Path

import yaml

from ..vairl.training_settings import get_default_training_settings
from .get_all_project_config import count_demo_folders
from .types import (
    Coord4D,
    NonNumericTopicFieldSelection,
    RoboGymAgentConfig,
    RoboGymDemoConfig,
    RoboGymDemoYaml,
    RoboGymProjectConfig,
    RoboGymProjectYaml,
    RoboGymTrainingYaml,
)
from .utils import (
    filter_populated_non_numeric_topic_fields,
    resolve_source_projects_dir,
    to_lower_snake_case,
    topic_to_data_folder_name,
)


def _format_agent_timestamp(dt: datetime) -> str:
    # 12-hour color with am/pm, zero padding
    hour_12 = dt.strftime("%I")
    ampm = dt.strftime("%p").lower()
    return f"{dt:%Y_%m_%d}_{hour_12}_{dt:%M}_{ampm}"


def _projects_root() -> Path:
    return resolve_source_projects_dir()


def _validate_topic_subtopics(
    *,
    field_name: str,
    topic_subtopics: object,
) -> None:
    if not isinstance(topic_subtopics, dict):
        raise ValueError(
            f"project['{field_name}'] must be a mapping of topic -> list[str].",
        )
    for topic, subtopics in topic_subtopics.items():
        if not isinstance(topic, str) or not topic.strip():
            raise ValueError(
                f"project['{field_name}'] contains an invalid topic name.",
            )
        if not isinstance(subtopics, list):
            raise ValueError(
                f"project['{field_name}'][{topic!r}] must be a list[str].",
            )
        for subtopic in subtopics:
            if not isinstance(subtopic, str) or not subtopic.strip():
                raise ValueError(
                    f"project['{field_name}'][{topic!r}] contains an invalid subtopic.",
                )


def _validate_non_numeric_topic_fields(
    *,
    field_name: str,
    topic_fields: object,
) -> None:
    if not isinstance(topic_fields, dict):
        raise ValueError(
            f"project['{field_name}'] must be a mapping of topic -> list[mapping].",
        )
    for topic, fields in topic_fields.items():
        if not isinstance(topic, str) or not topic.strip():
            raise ValueError(
                f"project['{field_name}'] contains an invalid topic name.",
            )
        if not isinstance(fields, list):
            raise ValueError(
                f"project['{field_name}'][{topic!r}] must be a list[mapping].",
            )
        for field in fields:
            if not isinstance(field, dict):
                raise ValueError(
                    f"project['{field_name}'][{topic!r}] must contain mappings.",
                )
            field_path = field.get("field_path")
            data_type = field.get("data_type")
            ros_type = field.get("ros_type")
            if not isinstance(field_path, str) or not field_path.strip():
                raise ValueError(
                    f"project['{field_name}'][{topic!r}] contains an invalid field_path.",
                )
            if data_type not in {"unordered_set", "image"}:
                raise ValueError(
                    f"project['{field_name}'][{topic!r}] contains an invalid data_type.",
                )
            if not isinstance(ros_type, str) or not ros_type.strip():
                raise ValueError(
                    f"project['{field_name}'][{topic!r}] contains an invalid ros_type.",
                )


def _validate_project_config_payload(project: RoboGymProjectYaml) -> None:
    required_fields = (
        "name",
        "world_file",
        "model_name",
        "random_spawn_space",
        "input_topics",
        "output_topics",
    )
    for field_name in required_fields:
        if field_name not in project:
            raise ValueError(f"project is missing required field {field_name!r}.")

    for field_name in ("name", "world_file", "model_name"):
        value = project[field_name]
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"project[{field_name!r}] must be a non-empty string.")

    random_spawn_space = project["random_spawn_space"]
    if not isinstance(random_spawn_space, dict):
        raise ValueError(
            "project['random_spawn_space'] must be a mapping.",
        )
    if not isinstance(random_spawn_space.get("enabled"), bool):
        raise ValueError("project['random_spawn_space']['enabled'] must be a bool.")
    for coord_field in ("coord1_4d", "coord2_4d"):
        coord = random_spawn_space.get(coord_field)
        if not isinstance(coord, list) or len(coord) != 4:
            raise ValueError(
                f"project['random_spawn_space']['{coord_field}'] must be a list[float] of length 4.",
            )
        for value in coord:
            if not isinstance(value, float):
                raise ValueError(
                    f"project['random_spawn_space']['{coord_field}'] must only contain floats.",
                )

    _validate_topic_subtopics(
        field_name="input_topics",
        topic_subtopics=project["input_topics"],
    )
    _validate_topic_subtopics(
        field_name="output_topics",
        topic_subtopics=project["output_topics"],
    )

    if "input_non_numeric_topics" in project:
        _validate_non_numeric_topic_fields(
            field_name="input_non_numeric_topics",
            topic_fields=project["input_non_numeric_topics"],
        )

    if "output_non_numeric_topics" in project:
        _validate_non_numeric_topic_fields(
            field_name="output_non_numeric_topics",
            topic_fields=project["output_non_numeric_topics"],
        )

    tensor_spec = project.get("tensor_spec")
    if tensor_spec is None:
        return
    if not isinstance(tensor_spec, dict):
        raise ValueError("project['tensor_spec'] must be a mapping when provided.")
    if (
        "ignored_input_features" in tensor_spec
        or "ignored_output_features" in tensor_spec
    ):
        raise ValueError(
            "project['tensor_spec'] must not include ignored_input_features or ignored_output_features.",
        )
    for key in ("input_features", "output_features", "input_dim", "output_dim"):
        if key not in tensor_spec:
            raise ValueError(f"project['tensor_spec'] is missing required key {key!r}.")

    for key in ("input_features", "output_features"):
        features = tensor_spec[key]
        if not isinstance(features, list) or any(
            not isinstance(feature, str) or not feature.strip() for feature in features
        ):
            raise ValueError(
                f"project['tensor_spec']['{key}'] must be a list[str].",
            )
    for key in ("input_dim", "output_dim"):
        value = tensor_spec[key]
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError(
                f"project['tensor_spec']['{key}'] must be a non-negative int.",
            )
    if tensor_spec["input_dim"] != len(tensor_spec["input_features"]):
        raise ValueError(
            "project['tensor_spec']['input_dim'] must match len(input_features).",
        )
    if tensor_spec["output_dim"] != len(tensor_spec["output_features"]):
        raise ValueError(
            "project['tensor_spec']['output_dim'] must match len(output_features).",
        )


def _normalized_topic_subtopics_for_compare(
    topic_subtopics: object,
) -> dict[str, list[str]]:
    if not isinstance(topic_subtopics, Mapping):
        return {}

    normalized: dict[str, list[str]] = {}
    for topic, fields in topic_subtopics.items():
        if not isinstance(topic, str) or not isinstance(fields, list):
            continue
        normalized[topic] = [str(field) for field in fields]
    return normalized


def _normalized_non_numeric_topic_fields_for_compare(
    topic_fields: object,
) -> dict[str, list[NonNumericTopicFieldSelection]]:
    if not isinstance(topic_fields, Mapping):
        return {}

    normalized: dict[str, list[NonNumericTopicFieldSelection]] = {}
    for topic, fields in topic_fields.items():
        if not isinstance(topic, str) or not isinstance(fields, list):
            continue

        normalized_fields: list[NonNumericTopicFieldSelection] = []
        for field in fields:
            if not isinstance(field, Mapping):
                continue

            field_path = field.get("field_path")
            data_type = field.get("data_type")
            ros_type = field.get("ros_type")
            if (
                not isinstance(field_path, str)
                or not isinstance(ros_type, str)
                or data_type not in {"unordered_set", "image"}
            ):
                continue

            normalized_fields.append(
                {
                    "field_path": field_path,
                    "data_type": data_type,
                    "ros_type": ros_type,
                },
            )

        if normalized_fields:
            normalized[topic] = normalized_fields

    return filter_populated_non_numeric_topic_fields(normalized)


def _project_topic_selection_signature(
    project_cfg: object,
) -> tuple[
    dict[str, list[str]],
    dict[str, list[str]],
    dict[str, list[NonNumericTopicFieldSelection]],
    dict[str, list[NonNumericTopicFieldSelection]],
]:
    if not isinstance(project_cfg, Mapping):
        return ({}, {}, {}, {})

    return (
        _normalized_topic_subtopics_for_compare(project_cfg.get("input_topics", {})),
        _normalized_topic_subtopics_for_compare(project_cfg.get("output_topics", {})),
        _normalized_non_numeric_topic_fields_for_compare(
            project_cfg.get("input_non_numeric_topics", {}),
        ),
        _normalized_non_numeric_topic_fields_for_compare(
            project_cfg.get("output_non_numeric_topics", {}),
        ),
    )


def _build_demo_config(
    *,
    name: str,
    sampling_rate: float,
    start_position: Coord4D,
) -> RoboGymDemoConfig:
    cfg_demo: RoboGymDemoYaml = {
        "name": name,
        "start_position": list(start_position),
        "sampling_rate": sampling_rate,
    }
    return {"robogym_demo": cfg_demo}


def _write_yaml_config(
    config_path: Path,
    cfg: RoboGymProjectConfig | RoboGymDemoConfig | RoboGymAgentConfig,
) -> None:
    with config_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)


def _build_agent_config(
    *,
    agent_name: str,
    num_demos: int,
    model_file_name: str,
    checkpoint_episode: int | None,
    training_settings: RoboGymTrainingYaml | None = None,
) -> RoboGymAgentConfig:
    cfg: RoboGymAgentConfig = {
        "robogym_agent": {
            "name": agent_name,
            "num_demos": num_demos,
            "model_file": model_file_name,
        },
    }
    if checkpoint_episode is not None:
        cfg["robogym_agent"]["checkpoint_episode"] = checkpoint_episode
    if training_settings is not None:
        cfg["robogym_agent"]["training_settings"] = dict(training_settings)
    return cfg


def _read_yaml_mapping(config_path: Path) -> dict[str, object]:
    if not config_path.is_file():
        return {}

    parsed = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if parsed is None:
        return {}
    if not isinstance(parsed, dict):
        raise ValueError(f"Config at {config_path} must be a mapping.")
    return dict(parsed)


METADATA_CSV_HEADERS = ("topic", "relative_path")
NUMERICAL_DATA_TOPIC = "/numerical"
ACTIONS_DATA_TOPIC = "/actions"
ACTIONS_CSV_HEADERS = (
    "delta_x",
    "delta_y",
    "delta_z",
    "delta_yaw",
    "x",
    "y",
    "z",
    "yaw",
)


def _relative_demo_path(demo_dir: Path, path: Path | str) -> str:
    candidate = Path(path)
    if candidate.is_absolute():
        try:
            relative_path = candidate.resolve().relative_to(demo_dir.resolve())
        except ValueError as e:
            raise ValueError(
                f"Path must be inside demo directory {demo_dir}: {candidate}",
            ) from e
    else:
        relative_path = candidate

    if ".." in relative_path.parts:
        raise ValueError(f"Relative path cannot escape demo directory: {relative_path}")
    return relative_path.as_posix()


def _ensure_demo_data_layout(demo_dir: Path) -> tuple[Path, Path, Path]:
    data_dir = demo_dir / "data"
    numerical_dir = data_dir / "numerical"
    actions_dir = data_dir / "actions"
    numerical_csv_path = numerical_dir / "data.csv"
    actions_csv_path = actions_dir / "data.csv"
    metadata_csv_path = demo_dir / "metadata.csv"

    numerical_dir.mkdir(parents=True, exist_ok=True)
    actions_dir.mkdir(parents=True, exist_ok=True)
    numerical_csv_path.touch(exist_ok=True)
    if (not actions_csv_path.exists()) or actions_csv_path.stat().st_size == 0:
        with actions_csv_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(ACTIONS_CSV_HEADERS)

    if (not metadata_csv_path.exists()) or metadata_csv_path.stat().st_size == 0:
        with metadata_csv_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(METADATA_CSV_HEADERS)
            writer.writerow(
                (
                    NUMERICAL_DATA_TOPIC,
                    Path("data/numerical/data.csv").as_posix(),
                ),
            )
            writer.writerow(
                (
                    ACTIONS_DATA_TOPIC,
                    Path("data/actions/data.csv").as_posix(),
                ),
            )

    return metadata_csv_path, numerical_csv_path, actions_csv_path


def append_demo_metadata_entry(
    demo_dir: Path,
    *,
    topic: str,
    relative_path: Path | str,
) -> None:
    """
    Append a metadata manifest row to <demo_dir>/metadata.csv.

    The metadata file is a simple index of topic -> relative file path.
    """
    normalized_topic = topic.strip()
    if not normalized_topic:
        raise ValueError("topic must be a non-empty string.")

    metadata_csv_path, _, _ = _ensure_demo_data_layout(demo_dir)
    normalized_relative_path = _relative_demo_path(demo_dir, relative_path)
    with metadata_csv_path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow((normalized_topic, normalized_relative_path))


def append_demo_numerical_row(
    project: RoboGymProjectYaml,
    demo: RoboGymDemoYaml,
    row: dict[str, int | float | complex],
) -> None:
    """
    Append one numerical sample row to <demo_dir>/data/numerical/data.csv.
    """

    # Get path for demo
    projects_dir = _projects_root()
    project_name = to_lower_snake_case(project["name"])
    demo_name = to_lower_snake_case(demo["name"])
    demo_dir = projects_dir / project_name / "demos" / demo_name

    _, numerical_csv_path, _ = _ensure_demo_data_layout(demo_dir)
    with numerical_csv_path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)


def initialize_demo_numerical_csv_headers(
    demo_dir: Path,
    headers: list[str],
) -> None:
    """
    Initialize numerical CSV headers when the file is empty.
    """
    if not headers:
        raise ValueError("headers must be a non-empty list of column names.")

    _, numerical_csv_path, _ = _ensure_demo_data_layout(demo_dir)
    if numerical_csv_path.stat().st_size != 0:
        return

    with numerical_csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)


def append_demo_action_row(
    demo_dir: Path,
    row: list[object],
) -> None:
    """
    Append one action sample row to <demo_dir>/data/actions/data.csv.
    """
    if not isinstance(row, list):
        raise ValueError("row must be a list of scalar values.")
    if len(row) != 4:
        raise ValueError(
            "action row must contain exactly 4 values: "
            "delta_x, delta_y, delta_z, delta_yaw.",
        )

    _, _, actions_csv_path = _ensure_demo_data_layout(demo_dir)
    with actions_csv_path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)


def write_demo_topic_payload_file(
    demo_dir: Path,
    *,
    topic: str,
    file_name: str,
    payload: bytes,
) -> Path:
    """
    Write a raw topic payload file under <demo_dir>/data/<topic_folder>/.

    This is useful for complex topic outputs (for example, YOLO detections)
    where CSV rows are not the desired representation.
    """
    if not file_name.strip():
        raise ValueError("file_name must be a non-empty string.")
    topic_data_dir = get_demo_topic_data_dir(demo_dir, topic=topic)
    file_path = topic_data_dir / file_name
    if file_path.exists():
        raise FileExistsError(f"Topic data file already exists: {file_path}")

    file_path.write_bytes(payload)
    append_demo_metadata_entry(
        demo_dir,
        topic=topic,
        relative_path=file_path,
    )
    return file_path


def get_demo_topic_data_dir(
    demo_dir: Path,
    *,
    topic: str,
) -> Path:
    """
    Resolve and create a topic-specific data folder under <demo_dir>/data/.

    Example:
        "/frontcam/image_raw" -> <demo_dir>/data/frontcam_image_raw
    """
    _ensure_demo_data_layout(demo_dir)
    topic_folder = topic_to_data_folder_name(topic)
    topic_data_dir = demo_dir / "data" / topic_folder
    topic_data_dir.mkdir(parents=True, exist_ok=True)
    return topic_data_dir


def write_demo_topic_png(
    demo_dir: Path,
    *,
    topic: str,
    file_name: str,
    png_data: bytes,
) -> Path:
    """
    Write a PNG image artifact and register it in metadata.csv.
    """
    if not file_name.strip():
        raise ValueError("file_name must be a non-empty string.")
    if not file_name.lower().endswith(".png"):
        raise ValueError("file_name must end with '.png'.")
    return write_demo_topic_payload_file(
        demo_dir,
        topic=topic,
        file_name=file_name,
        payload=png_data,
    )


def create_project_folder(
    project: RoboGymProjectYaml,
) -> Path:
    """
    Creates:
        <workspace_root>/src/mil_robogym/projects/<lower_snake_project_name>/config.yaml

    Uses the package source projects directory as the root.
    Returns the created project directory Path.
    """
    folder_name = to_lower_snake_case(project["name"])
    projects_dir = _projects_root()
    projects_dir.mkdir(parents=True, exist_ok=True)
    project_dir = projects_dir / folder_name
    if project_dir.exists():
        raise FileExistsError(f"Project folder already exists: {project_dir}")
    project_dir.mkdir(parents=True, exist_ok=False)
    (project_dir / "demos").mkdir(exist_ok=True)

    _validate_project_config_payload(project)
    cfg: RoboGymProjectConfig = {
        "robogym_project": project,
        "robogym_training": get_default_training_settings(),
    }
    _write_yaml_config(project_dir / "config.yaml", cfg)

    return project_dir


def edit_project(
    project: RoboGymProjectYaml,
    *,
    original_project_name: str | None = None,
) -> Path:
    """
    Edit an existing project's config.yaml using a RoboGymProjectYaml payload.

    Writes:
        <workspace_root>/src/mil_robogym/projects/<lower_snake_project_name>/config.yaml

    Returns the existing project directory Path.
    """
    projects_dir = _projects_root()

    current_name = original_project_name or project["name"]
    current_folder_name = to_lower_snake_case(current_name)
    project_dir = projects_dir / current_folder_name

    if not project_dir.exists() or not project_dir.is_dir():
        raise FileNotFoundError(f"Project folder does not exist: {project_dir}")

    _validate_project_config_payload(project)
    existing_cfg = _read_yaml_mapping(project_dir / "config.yaml")

    if count_demo_folders(project_dir / "demos") > 0:
        existing_project_cfg = existing_cfg.get("robogym_project", {})
        if _project_topic_selection_signature(existing_project_cfg) != (
            _project_topic_selection_signature(project)
        ):
            raise ValueError(
                "Cannot change project topic selections while demos already exist. "
                "Delete existing demos before editing topics or subtopics.",
            )

    target_folder_name = to_lower_snake_case(project["name"])
    target_project_dir = projects_dir / target_folder_name

    if target_project_dir != project_dir:
        if target_project_dir.exists():
            raise FileExistsError(
                f"Cannot rename project; target folder already exists: {target_project_dir}",
            )
        project_dir.rename(target_project_dir)
        project_dir = target_project_dir

    preserved_training_settings = existing_cfg.get("robogym_training")
    cfg: RoboGymProjectConfig = {"robogym_project": project}
    if isinstance(preserved_training_settings, dict):
        cfg["robogym_training"] = preserved_training_settings
    else:
        cfg["robogym_training"] = get_default_training_settings()
    _write_yaml_config(project_dir / "config.yaml", cfg)

    return project_dir


def edit_demo(
    project: RoboGymProjectYaml,
    demo: RoboGymDemoYaml,
    *,
    original_demo_name: str | None = None,
) -> Path:
    """
    Edit an existing demo's config.yaml using a RoboGymDemo payload.

    Writes:
        <workspace_root>/src/mil_robogym/projects/<lower_snake_project_name>/demos/<lower_snake_demo_name>/config.yaml

    Returns the existing demo directory Path
    """
    projects_dir = _projects_root()
    project_name = to_lower_snake_case(project["name"])
    demo_name = to_lower_snake_case(original_demo_name or demo["name"])
    demo_dir = projects_dir / project_name / "demos" / demo_name

    if not demo_dir.exists() or not demo_dir.is_dir():
        raise FileNotFoundError(f"Demo folder does not exist: {demo_dir}")

    target_folder_name = to_lower_snake_case(demo["name"])
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
            "name": demo["name"],
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
    num_demos: int = 0,
    created_at: datetime | None,
    model_file_name: str = "weights.pt",
    agent_name: str | None = None,
    checkpoint_episode: int | None = None,
    training_settings: RoboGymTrainingYaml | None = None,
) -> Path:
    """
    Create a timestamped agent folder under <project_dir>/agents/

    Writes:
      - config.yaml
      - model artifact copied from trained_model_path
      - training_metrics.csv
      - metrics/*.png (one plot per metric series)

    Raises:
      FileExistsError if the timestamp folder already exists
      FileNotFoundError if trained_model_path does not exist
      ValueError if training_metrics is empty or lengths mismatch
    """
    if not trained_model_path.exists():
        raise FileNotFoundError(f"trained_model_path not found: {trained_model_path}")

    if not model_file_name.strip():
        raise ValueError("model_file_name must be a non-empty string")

    if not training_metrics:
        raise ValueError("training_metrics is empty")

    # Validate metric lengths are consistent
    lengths = {len(v) for v in training_metrics.values()}
    if len(lengths) != 1:
        raise ValueError(f"training_metrics series lengths mismatch: {sorted(lengths)}")
    n = lengths.pop()

    dt = created_at or datetime.now()
    resolved_agent_name = agent_name or _format_agent_timestamp(dt)

    agents_dir = project_dir / "agents"
    agents_dir.mkdir(parents=True, exist_ok=True)

    agent_dir = agents_dir / resolved_agent_name
    if agent_dir.exists():
        raise FileExistsError(f"Agent folder already exists: {agent_dir}")
    agent_dir.mkdir(parents=True, exist_ok=False)

    # 1) config.yaml
    cfg = _build_agent_config(
        agent_name=resolved_agent_name,
        num_demos=num_demos,
        model_file_name=model_file_name,
        checkpoint_episode=checkpoint_episode,
        training_settings=training_settings,
    )
    with (agent_dir / "config.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    # 2) model artifact
    shutil.copyfile(trained_model_path, agent_dir / model_file_name)

    # 3) metrics CSV
    csv_path = agent_dir / "training_metrics.csv"
    headers = ["index", *training_metrics.keys()]
    with csv_path.open("w", encoding="utf-8") as f:
        f.write(",".join(headers) + "\n")
        for i in range(n):
            row = [str(i)] + [str(training_metrics[k][i]) for k in training_metrics]
            f.write(",".join(row) + "\n")

    # 4) plots
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
    project: RoboGymProjectYaml,
    *,
    name: str,
    sampling_rate: float,
    start_position: Coord4D | None = None,
) -> tuple[Path, RoboGymDemoConfig]:
    """
    Create a demo folder under <project_dir>/demos/ with a config.yaml.

    Creates:
        <project_dir>/demos/<lower_snake_demo_name>/config.yaml
        <project_dir>/demos/<lower_snake_demo_name>/metadata.csv
        <project_dir>/demos/<lower_snake_demo_name>/data/numerical/data.csv
        <project_dir>/demos/<lower_snake_demo_name>/data/actions/data.csv

    If start_position is None, defaults to (0.0, 0.0, 0.0, 0.0).
    Returns the created demo directory Path.
    """
    projects_dir = _projects_root()
    project_name = to_lower_snake_case(project["name"])
    project_dir = projects_dir / project_name

    demos_dir = project_dir / "demos"
    demos_dir.mkdir(parents=True, exist_ok=True)

    folder_name = to_lower_snake_case(name)
    demo_dir = demos_dir / folder_name

    if demo_dir.exists():
        raise FileExistsError(f"Demo folder already exists: {demo_dir}")

    demo_dir.mkdir(parents=True, exist_ok=False)

    if start_position is None:
        start_position = (0.0, 0.0, 0.0, 0.0)

    config_path = demo_dir / "config.yaml"
    cfg = _build_demo_config(
        name=name,
        sampling_rate=sampling_rate,
        start_position=start_position,
    )
    _write_yaml_config(config_path, cfg)
    _ensure_demo_data_layout(demo_dir)

    # Flatten project input topics
    initialize_demo_numerical_csv_headers(
        demo_dir,
        project["tensor_spec"]["input_features"],
    )

    return demo_dir, cfg


def get_demo_dir_path(project: RoboGymProjectYaml, demo: RoboGymDemoYaml) -> Path:
    """
    Get the demo dir path based on the project and demo yamls.
    """
    projects_dir = _projects_root()
    project_name = to_lower_snake_case(project["name"])
    demo_name = to_lower_snake_case(demo["name"])
    demo_dir = projects_dir / project_name / "demos" / demo_name

    return demo_dir


def get_project_dir_path(project: RoboGymProjectYaml) -> Path:
    """
    Get the project dir path based on the project yaml.
    """
    projects_dir = _projects_root()
    project_name = to_lower_snake_case(project["name"])
    project_dir = projects_dir / project_name

    return project_dir


def get_training_project_dir_paths(project: RoboGymProjectYaml) -> list[Path]:
    """
    Get project directory paths for generated training artifacts.

    Source-only mode returns a single project directory path.
    """
    project_name = to_lower_snake_case(project["name"])
    return [_projects_root() / project_name]


def get_training_project_dir_path(project: RoboGymProjectYaml) -> Path:
    """
    Get the preferred project directory for generated training artifacts.
    """
    return get_training_project_dir_paths(project)[0]


def update_project_training_settings(
    project: RoboGymProjectYaml,
    training_settings: RoboGymTrainingYaml,
) -> list[Path]:
    """
    Update the persisted training settings for a project.
    """
    updated_paths: list[Path] = []
    for project_dir in get_training_project_dir_paths(project):
        config_path = project_dir / "config.yaml"
        cfg = _read_yaml_mapping(config_path)
        cfg["robogym_project"] = project
        cfg["robogym_training"] = dict(training_settings)
        _write_yaml_config(config_path, cfg)
        updated_paths.append(config_path)
    return updated_paths
