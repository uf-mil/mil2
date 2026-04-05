"""Tests for asynchronous training metrics persistence."""

from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path

import pytest

from mil_robogym.vairl.metrics import TrainingMetricsSession


class DummyGenerator:
    """Minimal generator stub that serializes model bytes to disk."""

    def __init__(self) -> None:
        self.saved_paths: list[Path] = []

    def save(self, path: str) -> None:
        model_path = Path(path)
        model_path.write_bytes(b"generator-artifact")
        self.saved_paths.append(model_path)


def _build_training_settings() -> dict[str, int | float | None]:
    return {
        "num_episodes": 12,
        "rollout_steps": 32,
        "generator_learning_rate": 1e-3,
        "discriminator_learning_rate": 3e-3,
        "z_size": 6,
        "e_hidden_size": 128,
        "i_c": 0.5,
        "beta_step_size": 1e-3,
        "gamma": 0.99,
        "save_every": 2,
        "seed": 42,
        "max_step_count": None,
        "expert_noise_std": 1e-4,
    }


def _record_episode(
    session: TrainingMetricsSession,
    *,
    episode: int,
    reward_mean: float,
    extra_metrics: dict[str, float] | None = None,
) -> None:
    session.record_episode(
        episode=episode,
        reward_mean=reward_mean,
        reward_std=0.1 * episode,
        disc_stats={
            "loss": 1.0 / episode,
            "kl": 0.05 * episode,
            "beta": 0.01 * episode,
        },
        extra_metrics=extra_metrics,
    )


def test_training_metrics_session_snapshots_checkpoint_and_final_artifacts(
    tmp_path: Path,
) -> None:
    """Checkpoint saves use the metric snapshot from enqueue time."""
    session = TrainingMetricsSession()
    generator = DummyGenerator()
    source_project_dir = tmp_path / "source_project"
    share_project_dir = tmp_path / "share_project"

    _record_episode(
        session,
        episode=1,
        reward_mean=1.0,
        extra_metrics={"gen_time_fps": 166.0},
    )
    _record_episode(
        session,
        episode=2,
        reward_mean=2.0,
        extra_metrics={"gen_time_fps": 167.0},
    )
    checkpoint_future = session.enqueue_agent_save(
        generator=generator,
        project_dirs=[source_project_dir, share_project_dir],
        num_demos=2,
        created_at=datetime(2026, 3, 29, 14, 0),
        model_file_name="generator_model.zip",
        agent_name="2026_03_29_02_00_pm_ep_0002",
        checkpoint_episode=2,
        training_settings=_build_training_settings(),
    )

    _record_episode(
        session,
        episode=3,
        reward_mean=3.0,
        extra_metrics={"gen_time_fps": 168.0},
    )
    final_future = session.enqueue_agent_save(
        generator=generator,
        project_dirs=[source_project_dir, share_project_dir],
        num_demos=2,
        created_at=datetime(2026, 3, 29, 14, 1),
        model_file_name="generator_model.zip",
        agent_name="2026_03_29_02_01_pm_final",
        training_settings=_build_training_settings(),
    )

    session.close()

    checkpoint_agent_dirs = checkpoint_future.result()
    final_agent_dirs = final_future.result()

    assert len(generator.saved_paths) == 2
    assert [path.name for path in checkpoint_agent_dirs] == [
        "2026_03_29_02_00_pm_ep_0002",
        "2026_03_29_02_00_pm_ep_0002",
    ]
    assert [path.name for path in final_agent_dirs] == [
        "2026_03_29_02_01_pm_final",
        "2026_03_29_02_01_pm_final",
    ]

    checkpoint_rows = (
        (
            source_project_dir
            / "agents"
            / "2026_03_29_02_00_pm_ep_0002"
            / "training_metrics.csv"
        )
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    )
    final_rows = (
        (
            source_project_dir
            / "agents"
            / "2026_03_29_02_01_pm_final"
            / "training_metrics.csv"
        )
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    )

    assert "gen_time_fps" in checkpoint_rows[0]
    assert checkpoint_rows[-1].endswith(",167.0")
    assert final_rows[-1].endswith(",168.0")
    assert len(checkpoint_rows) == 3
    assert len(final_rows) == 4
    assert (
        source_project_dir
        / "agents"
        / "2026_03_29_02_01_pm_final"
        / "metrics"
        / "reward_mean.png"
    ).is_file()


def test_training_metrics_session_surfaces_worker_failures_on_close(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """Background save failures still fail the caller on close."""
    session = TrainingMetricsSession()
    generator = DummyGenerator()

    _record_episode(session, episode=1, reward_mean=1.0)

    def _raise_on_save(*args, **kwargs):
        raise ValueError("save exploded")

    monkeypatch.setattr("mil_robogym.vairl.metrics.create_agent_folder", _raise_on_save)

    future = session.enqueue_agent_save(
        generator=generator,
        project_dirs=[tmp_path / "project"],
        num_demos=1,
        created_at=datetime(2026, 3, 29, 14, 2),
        model_file_name="generator_model.zip",
        agent_name="2026_03_29_02_02_pm_final",
        training_settings=_build_training_settings(),
    )

    with pytest.raises(
        RuntimeError,
        match="Asynchronous training metrics save failed.",
    ):
        session.close()

    with pytest.raises(ValueError, match="save exploded"):
        future.result()


def test_training_metrics_session_backfills_missing_dynamic_metric_values() -> None:
    """Dynamic metric columns stay aligned when later episodes omit a value."""
    session = TrainingMetricsSession()

    _record_episode(
        session,
        episode=1,
        reward_mean=1.0,
        extra_metrics={"gen_time_fps": 166.0},
    )
    _record_episode(session, episode=2, reward_mean=2.0)

    snapshot = session.snapshot()
    session.close()

    assert snapshot["episode"] == [1.0, 2.0]
    assert snapshot["gen_time_fps"][0] == 166.0
    assert math.isnan(snapshot["gen_time_fps"][1])
