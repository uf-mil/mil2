from __future__ import annotations

from collections.abc import Mapping

from mil_robogym.data_collection.types import RoboGymTrainingYaml

DEFAULT_TRAINING_SETTINGS: RoboGymTrainingYaml = {
    "num_episodes": 500,
    "rollout_steps": 2048,
    "generator_learning_rate": 1e-3,
    "discriminator_learning_rate": 3e-3,
    "z_size": 6,
    "e_hidden_size": 128,
    "i_c": 0.5,
    "beta_step_size": 1e-3,
    "gamma": 0.99,
    "save_every": 10,
    "seed": 42,
    "max_step_count": None,
    "expert_noise_std": 1e-4,
}


def get_default_training_settings() -> RoboGymTrainingYaml:
    """Return a copy of the default training settings."""
    return dict(DEFAULT_TRAINING_SETTINGS)


def normalize_training_settings(
    settings: Mapping[str, object] | None = None,
) -> RoboGymTrainingYaml:
    """Validate and coerce a partial training-settings mapping."""
    merged = get_default_training_settings()
    raw_settings = settings or {}

    merged["num_episodes"] = _coerce_int(
        raw_settings.get("num_episodes", merged["num_episodes"]),
        field_name="num_episodes",
        minimum=1,
    )
    merged["rollout_steps"] = _coerce_int(
        raw_settings.get("rollout_steps", merged["rollout_steps"]),
        field_name="rollout_steps",
        minimum=1,
    )
    merged["generator_learning_rate"] = _coerce_float(
        raw_settings.get(
            "generator_learning_rate",
            merged["generator_learning_rate"],
        ),
        field_name="generator_learning_rate",
        minimum=0.0,
        inclusive_min=False,
    )
    merged["discriminator_learning_rate"] = _coerce_float(
        raw_settings.get(
            "discriminator_learning_rate",
            merged["discriminator_learning_rate"],
        ),
        field_name="discriminator_learning_rate",
        minimum=0.0,
        inclusive_min=False,
    )
    merged["z_size"] = _coerce_int(
        raw_settings.get("z_size", merged["z_size"]),
        field_name="z_size",
        minimum=1,
    )
    merged["e_hidden_size"] = _coerce_int(
        raw_settings.get("e_hidden_size", merged["e_hidden_size"]),
        field_name="e_hidden_size",
        minimum=1,
    )
    merged["i_c"] = _coerce_float(
        raw_settings.get("i_c", merged["i_c"]),
        field_name="i_c",
        minimum=0.0,
        inclusive_min=False,
    )
    merged["beta_step_size"] = _coerce_float(
        raw_settings.get("beta_step_size", merged["beta_step_size"]),
        field_name="beta_step_size",
        minimum=0.0,
        inclusive_min=False,
    )
    merged["gamma"] = _coerce_float(
        raw_settings.get("gamma", merged["gamma"]),
        field_name="gamma",
        minimum=0.0,
        maximum=1.0,
        inclusive_min=False,
        inclusive_max=True,
    )
    merged["save_every"] = _coerce_int(
        raw_settings.get("save_every", merged["save_every"]),
        field_name="save_every",
        minimum=0,
    )
    merged["seed"] = _coerce_int(
        raw_settings.get("seed", merged["seed"]),
        field_name="seed",
        minimum=0,
    )
    merged["max_step_count"] = _coerce_optional_int(
        raw_settings.get("max_step_count", merged["max_step_count"]),
        field_name="max_step_count",
        minimum=1,
    )
    merged["expert_noise_std"] = _coerce_float(
        raw_settings.get("expert_noise_std", merged["expert_noise_std"]),
        field_name="expert_noise_std",
        minimum=0.0,
        inclusive_min=True,
    )

    return merged


def _coerce_optional_int(
    value: object,
    *,
    field_name: str,
    minimum: int,
) -> int | None:
    if value is None:
        return None
    if isinstance(value, str) and value.strip().lower() in {"", "none", "auto"}:
        return None
    return _coerce_int(value, field_name=field_name, minimum=minimum)


def _coerce_int(value: object, *, field_name: str, minimum: int) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{field_name} must be an integer.")
    try:
        parsed = int(str(value).strip())
    except (TypeError, ValueError) as e:
        raise ValueError(f"{field_name} must be an integer.") from e
    if parsed < minimum:
        raise ValueError(f"{field_name} must be >= {minimum}.")
    return parsed


def _coerce_float(
    value: object,
    *,
    field_name: str,
    minimum: float,
    inclusive_min: bool,
    maximum: float | None = None,
    inclusive_max: bool = True,
) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field_name} must be a number.")
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError) as e:
        raise ValueError(f"{field_name} must be a number.") from e

    if inclusive_min:
        if parsed < minimum:
            raise ValueError(f"{field_name} must be >= {minimum}.")
    elif parsed <= minimum:
        raise ValueError(f"{field_name} must be > {minimum}.")

    if maximum is not None:
        if inclusive_max:
            if parsed > maximum:
                raise ValueError(f"{field_name} must be <= {maximum}.")
        elif parsed >= maximum:
            raise ValueError(f"{field_name} must be < {maximum}.")

    return parsed
