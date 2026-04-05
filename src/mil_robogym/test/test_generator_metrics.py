"""Tests for extracting generator-side training metrics from the logger."""

from __future__ import annotations

from types import SimpleNamespace

from mil_robogym.vairl.generator_metrics import extract_latest_generator_metrics


def test_extract_latest_generator_metrics_reads_and_clears_mean_keys() -> None:
    """Generator metric extraction returns mapped values and resets the logger state."""
    default_logger = SimpleNamespace(
        name_to_value={
            "mean/gen/train/value_loss": 1.34,
            "mean/gen/time/fps": 166,
            "mean/gen/rollout/ep_rew_mean": -73,
        },
        name_to_count={
            "mean/gen/train/value_loss": 1,
            "mean/gen/time/fps": 1,
            "mean/gen/rollout/ep_rew_mean": 1,
        },
        name_to_excluded={},
    )
    logger = SimpleNamespace(default_logger=default_logger)

    extracted = extract_latest_generator_metrics(logger)

    assert extracted == {
        "gen_rollout_ep_rew_mean": -73.0,
        "gen_time_fps": 166.0,
        "gen_train_value_loss": 1.34,
    }
    assert default_logger.name_to_value == {}
    assert default_logger.name_to_count == {}
