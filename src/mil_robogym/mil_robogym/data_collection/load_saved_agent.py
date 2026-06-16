from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, cast

import numpy as np
import yaml

from mil_robogym.vairl.observation_preprocessor import ObservationPreprocessor

from .filesystem import get_training_project_dir_path
from .types import RoboGymProjectYaml, RoboGymTrainingYaml


@dataclass(frozen=True, slots=True)
class SavedAgentHandle:
    """Validated file-system handle for a saved agent artifact."""

    project_name: str
    agent_name: str
    agent_dir: Path
    config_path: Path
    model_path: Path
    preprocessor_path: Path | None
    metrics_csv_path: Path | None
    model_file_name: str
    num_demos: int
    checkpoint_episode: int | None
    training_settings: RoboGymTrainingYaml | None

    @property
    def is_final(self) -> bool:
        return self.checkpoint_episode is None


@dataclass(slots=True)
class LoadedAgent:
    """Callable saved-agent wrapper around a loaded generator model."""

    handle: SavedAgentHandle
    model: Any
    input_size: int
    encoded_input_size: int
    output_size: int
    preprocessor: ObservationPreprocessor | None = None

    def predict(
        self,
        observation: np.ndarray | list[object],
        *,
        deterministic: bool = True,
    ) -> np.ndarray:
        if self.preprocessor is not None:
            observation_array = self.preprocessor.encode_state(observation)
        else:
            observation_array = np.asarray(observation, dtype=np.float32)
            expected_shape = (self.input_size,)
            if observation_array.shape != expected_shape:
                raise ValueError(
                    "Saved agent observation shape does not match the expected size: "
                    f"{observation_array.shape} != {expected_shape}",
                )

        expected_encoded_shape = (self.encoded_input_size,)
        if observation_array.shape != expected_encoded_shape:
            raise ValueError(
                "Saved agent encoded observation shape does not match the expected size: "
                f"{observation_array.shape} != {expected_encoded_shape}",
            )

        action, _state = self.model.predict(
            observation_array,
            deterministic=deterministic,
        )
        action_array = np.asarray(action, dtype=np.float32).reshape(-1)

        expected_action_shape = (self.output_size,)
        if action_array.shape != expected_action_shape:
            raise ValueError(
                "Saved agent action shape does not match the expected size: "
                f"{action_array.shape} != {expected_action_shape}",
            )

        return action_array


def load_saved_agent(project: Mapping[str, Any], agent_name: str) -> SavedAgentHandle:
    """Load and validate one saved agent folder for the given project."""
    project_yaml = _coerce_project_yaml(project)
    resolved_agent_name = _coerce_non_empty_string(
        field_name="agent_name",
        value=agent_name,
    )

    agent_dir = (
        get_training_project_dir_path(project_yaml) / "agents" / resolved_agent_name
    )
    if not agent_dir.is_dir():
        raise FileNotFoundError(f"Saved agent directory does not exist: {agent_dir}")

    config_path = agent_dir / "config.yaml"
    config = _read_yaml_mapping(config_path)

    agent_cfg_raw = config.get("robogym_agent")
    if not isinstance(agent_cfg_raw, Mapping):
        raise ValueError(
            f"Config for saved agent '{resolved_agent_name}' is missing robogym_agent.",
        )

    config_agent_name = _coerce_non_empty_string(
        field_name="robogym_agent.name",
        value=agent_cfg_raw.get("name"),
    )
    if config_agent_name != resolved_agent_name:
        raise ValueError(
            "Saved agent config name does not match the selected agent directory: "
            f"{config_agent_name!r} != {resolved_agent_name!r}",
        )

    num_demos = _coerce_int(
        field_name="robogym_agent.num_demos",
        value=agent_cfg_raw.get("num_demos"),
    )
    model_file_name = _coerce_non_empty_string(
        field_name="robogym_agent.model_file",
        value=agent_cfg_raw.get("model_file"),
    )
    _validate_relative_agent_path(model_file_name)
    preprocessor_file_raw = agent_cfg_raw.get("preprocessor_file")
    if preprocessor_file_raw is None:
        preprocessor_path = None
    else:
        preprocessor_file_name = _coerce_non_empty_string(
            field_name="robogym_agent.preprocessor_file",
            value=preprocessor_file_raw,
        )
        _validate_relative_agent_path(preprocessor_file_name)
        preprocessor_path = agent_dir / preprocessor_file_name
        if not preprocessor_path.is_file():
            raise FileNotFoundError(
                f"Saved preprocessor file does not exist: {preprocessor_path}",
            )

    checkpoint_episode_raw = agent_cfg_raw.get("checkpoint_episode")
    checkpoint_episode = (
        _coerce_int(
            field_name="robogym_agent.checkpoint_episode",
            value=checkpoint_episode_raw,
        )
        if checkpoint_episode_raw is not None
        else None
    )

    training_settings_raw = agent_cfg_raw.get("training_settings")
    if training_settings_raw is None:
        training_settings = None
    elif isinstance(training_settings_raw, Mapping):
        training_settings = cast(RoboGymTrainingYaml, dict(training_settings_raw))
    else:
        raise ValueError(
            "robogym_agent.training_settings must be a mapping when provided.",
        )

    model_path = agent_dir / model_file_name
    if not model_path.is_file():
        raise FileNotFoundError(f"Saved model file does not exist: {model_path}")

    metrics_csv_path = agent_dir / "training_metrics.csv"
    if not metrics_csv_path.is_file():
        metrics_csv_path = None

    return SavedAgentHandle(
        project_name=project_yaml["name"],
        agent_name=resolved_agent_name,
        agent_dir=agent_dir,
        config_path=config_path,
        model_path=model_path,
        preprocessor_path=preprocessor_path,
        metrics_csv_path=metrics_csv_path,
        model_file_name=model_file_name,
        num_demos=num_demos,
        checkpoint_episode=checkpoint_episode,
        training_settings=training_settings,
    )


def load_saved_agent_model(project: Mapping[str, Any], agent_name: str) -> LoadedAgent:
    """Load a saved agent and return a callable predictor wrapper."""
    handle = load_saved_agent(project, agent_name)
    model = _load_policy_model(handle.model_path)
    encoded_input_size = _resolve_model_input_size(project, model)
    preprocessor = (
        ObservationPreprocessor.load(handle.preprocessor_path)
        if handle.preprocessor_path is not None
        else None
    )
    if (
        preprocessor is not None
        and preprocessor.encoded_input_size != encoded_input_size
    ):
        raise ValueError(
            "Saved agent preprocessor output size does not match the policy input size: "
            f"{preprocessor.encoded_input_size} != {encoded_input_size}",
        )
    input_size = (
        preprocessor.raw_input_size
        if preprocessor is not None
        else _resolve_input_size(project)
    )
    output_size = _resolve_output_size(project, model)

    return LoadedAgent(
        handle=handle,
        model=model,
        input_size=input_size,
        encoded_input_size=encoded_input_size,
        output_size=output_size,
        preprocessor=preprocessor,
    )


def _coerce_project_yaml(project: Mapping[str, Any]) -> RoboGymProjectYaml:
    project_yaml_raw = project.get("robogym_project", project)
    if not isinstance(project_yaml_raw, Mapping):
        raise ValueError("Project payload must be a mapping.")

    project_name = _coerce_non_empty_string(
        field_name="robogym_project.name",
        value=project_yaml_raw.get("name"),
    )
    return cast(RoboGymProjectYaml, {"name": project_name})


def _coerce_non_empty_string(*, field_name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string.")
    return value.strip()


def _coerce_int(*, field_name: str, value: object) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"{field_name} must be an int.") from e


def _validate_relative_agent_path(path_value: str) -> None:
    path = Path(path_value)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(
            "Saved agent artifact paths must stay inside the agent folder.",
        )


def _read_yaml_mapping(config_path: Path) -> dict[str, object]:
    if not config_path.is_file():
        raise FileNotFoundError(f"Saved agent config does not exist: {config_path}")

    try:
        parsed = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as e:
        raise ValueError(
            f"Saved agent config could not be parsed: {config_path}",
        ) from e

    if not isinstance(parsed, dict):
        raise ValueError(f"Saved agent config must be a mapping: {config_path}")
    return dict(parsed)


def _resolve_input_size(project: Mapping[str, Any]) -> int:
    tensor_spec = _require_tensor_spec(project)
    input_dim = tensor_spec.get("input_dim")
    if input_dim is not None:
        return _coerce_int(
            field_name="robogym_project.tensor_spec.input_dim",
            value=input_dim,
        )

    input_features = tensor_spec.get("input_features")
    if isinstance(input_features, list):
        return len(input_features)

    raise ValueError("Project tensor_spec must define input_dim or input_features.")


def _resolve_output_size(project: Mapping[str, Any], model: Any) -> int:
    action_space = getattr(model, "action_space", None)
    action_shape = getattr(action_space, "shape", None)
    if (
        isinstance(action_shape, tuple)
        and len(action_shape) == 1
        and isinstance(action_shape[0], (int, np.integer))
        and int(action_shape[0]) > 0
    ):
        return int(action_shape[0])

    tensor_spec = _require_tensor_spec(project)
    output_dim = tensor_spec.get("output_dim")
    if output_dim is not None:
        return _coerce_int(
            field_name="robogym_project.tensor_spec.output_dim",
            value=output_dim,
        )

    output_features = tensor_spec.get("output_features")
    if isinstance(output_features, list):
        return len(output_features)

    raise ValueError(
        "Unable to determine saved agent output size from the model or tensor_spec.",
    )


def _resolve_model_input_size(project: Mapping[str, Any], model: Any) -> int:
    observation_space = getattr(model, "observation_space", None)
    observation_shape = getattr(observation_space, "shape", None)
    if (
        isinstance(observation_shape, tuple)
        and len(observation_shape) == 1
        and isinstance(observation_shape[0], (int, np.integer))
        and int(observation_shape[0]) > 0
    ):
        return int(observation_shape[0])

    return _resolve_input_size(project)


def _require_tensor_spec(project: Mapping[str, Any]) -> Mapping[str, Any]:
    project_yaml_raw = project.get("robogym_project", project)
    if not isinstance(project_yaml_raw, Mapping):
        raise ValueError("Project payload must be a mapping.")

    tensor_spec = project_yaml_raw.get("tensor_spec")
    if not isinstance(tensor_spec, Mapping):
        raise ValueError("Project payload must include a tensor_spec mapping.")
    return tensor_spec


def _load_policy_model(model_path: Path) -> Any:
    from sb3_contrib import TRPO

    return TRPO.load(str(model_path))
