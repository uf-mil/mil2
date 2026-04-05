from __future__ import annotations

from collections.abc import Mapping
from typing import Any

GENERATOR_LOGGER_KEY_MAP: dict[str, str] = {
    "mean/gen/rollout/ep_len_mean": "gen_rollout_ep_len_mean",
    "mean/gen/rollout/ep_rew_mean": "gen_rollout_ep_rew_mean",
    "mean/gen/rollout/ep_rew_wrapped_mean": "gen_rollout_ep_rew_wrapped_mean",
    "mean/gen/time/fps": "gen_time_fps",
    "mean/gen/time/iterations": "gen_time_iterations",
    "mean/gen/time/time_elapsed": "gen_time_time_elapsed",
    "mean/gen/time/total_timesteps": "gen_time_total_timesteps",
    "mean/gen/train/explained_variance": "gen_train_explained_variance",
    "mean/gen/train/is_line_search_success": "gen_train_is_line_search_success",
    "mean/gen/train/kl_divergence_loss": "gen_train_kl_divergence_loss",
    "mean/gen/train/learning_rate": "gen_train_learning_rate",
    "mean/gen/train/n_updates": "gen_train_n_updates",
    "mean/gen/train/policy_objective": "gen_train_policy_objective",
    "mean/gen/train/std": "gen_train_std",
    "mean/gen/train/value_loss": "gen_train_value_loss",
}


def extract_latest_generator_metrics(logger: Any) -> dict[str, float]:
    """Read the latest generator metrics from the adversarial logger and reset them."""
    default_logger = getattr(logger, "default_logger", None)
    if default_logger is None:
        return {}

    name_to_value: Mapping[str, object] = getattr(default_logger, "name_to_value", {})
    name_to_count = getattr(default_logger, "name_to_count", None)
    name_to_excluded = getattr(default_logger, "name_to_excluded", None)

    extracted: dict[str, float] = {}
    for logger_key, metric_name in GENERATOR_LOGGER_KEY_MAP.items():
        raw_value = name_to_value.get(logger_key)
        if raw_value is None:
            continue
        try:
            extracted[metric_name] = float(raw_value)
        except (TypeError, ValueError):
            continue

        if hasattr(name_to_value, "pop"):
            name_to_value.pop(logger_key, None)
        if hasattr(name_to_count, "pop"):
            name_to_count.pop(logger_key, None)
        if hasattr(name_to_excluded, "pop"):
            name_to_excluded.pop(logger_key, None)

    return extracted
