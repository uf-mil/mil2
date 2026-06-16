from __future__ import annotations

import queue
import tempfile
import threading
from collections.abc import Mapping
from concurrent.futures import Future
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Protocol, Sequence
from uuid import uuid4

from mil_robogym.data_collection.filesystem import create_agent_folder
from mil_robogym.data_collection.types import RoboGymTrainingYaml

TrainingMetrics = dict[str, list[float]]
MetricsEventCallback = Callable[[dict[str, object]], None]
_STOP_WORKER = object()
_BASE_METRIC_NAMES = (
    "episode",
    "reward_mean",
    "reward_std",
    "disc_loss",
    "disc_kl",
    "disc_beta",
)


class _ModelSaver(Protocol):
    def save(self, path: str) -> None: ...


@dataclass(slots=True)
class _AgentSaveRequest:
    project_dirs: tuple[Path, ...]
    trained_model_path: Path
    preprocessor_artifact_path: Path | None
    training_metrics: TrainingMetrics
    num_demos: int
    created_at: datetime
    model_file_name: str
    agent_name: str
    checkpoint_episode: int | None
    preprocessor_file_name: str | None
    training_settings: RoboGymTrainingYaml | None
    future: Future[list[Path]]


class TrainingMetricsSession:
    """Collect training metrics and save metric artifacts on a worker thread."""

    def __init__(
        self,
        *,
        save_callback: MetricsEventCallback | None = None,
    ) -> None:
        self._metrics: TrainingMetrics = {
            "episode": [],
            "reward_mean": [],
            "reward_std": [],
            "disc_loss": [],
            "disc_kl": [],
            "disc_beta": [],
        }
        self._metrics_lock = threading.Lock()
        self._failure_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._worker_failure: Exception | None = None
        self._closed = False
        self._save_callback = save_callback

        self._temp_dir = tempfile.TemporaryDirectory(prefix="mil_robogym_metrics_")
        self._temp_dir_path = Path(self._temp_dir.name)
        self._save_queue: queue.Queue[_AgentSaveRequest | object] = queue.Queue()
        self._worker = threading.Thread(
            target=self._run_worker,
            name="mil_robogym_metrics_worker",
            daemon=True,
        )
        self._worker.start()

    def record_episode(
        self,
        *,
        episode: int,
        reward_mean: float,
        reward_std: float,
        disc_stats: dict[str, float],
        extra_metrics: Mapping[str, float] | None = None,
    ) -> None:
        """Append one episode of scalar metrics."""
        self._raise_if_failed()
        with self._metrics_lock:
            previous_episode_count = len(self._metrics["episode"])
            self._metrics["episode"].append(float(episode))
            self._metrics["reward_mean"].append(float(reward_mean))
            self._metrics["reward_std"].append(float(reward_std))
            self._metrics["disc_loss"].append(float(disc_stats["loss"]))
            self._metrics["disc_kl"].append(float(disc_stats["kl"]))
            self._metrics["disc_beta"].append(float(disc_stats["beta"]))
            self._append_extra_metrics(
                previous_episode_count=previous_episode_count,
                extra_metrics=extra_metrics or {},
            )

    def snapshot(self) -> TrainingMetrics:
        """Return an immutable-by-convention copy for background persistence."""
        with self._metrics_lock:
            return {name: list(series) for name, series in self._metrics.items()}

    def enqueue_agent_save(
        self,
        *,
        generator: _ModelSaver,
        preprocessor: _ModelSaver | None = None,
        project_dirs: Sequence[Path],
        num_demos: int,
        created_at: datetime,
        model_file_name: str,
        agent_name: str,
        checkpoint_episode: int | None = None,
        preprocessor_file_name: str | None = None,
        training_settings: RoboGymTrainingYaml | None = None,
    ) -> Future[list[Path]]:
        """Serialize the model now and persist the heavier artifacts asynchronously."""
        self._raise_if_closed()
        self._raise_if_failed()

        trained_model_path = self._reserve_model_path(
            agent_name=agent_name,
            model_file_name=model_file_name,
        )
        preprocessor_artifact_path: Path | None = None
        if (preprocessor is None) != (preprocessor_file_name is None):
            raise ValueError(
                "preprocessor and preprocessor_file_name must be provided together.",
            )
        if preprocessor is not None and preprocessor_file_name is not None:
            preprocessor_artifact_path = self._reserve_model_path(
                agent_name=f"{agent_name}_preprocessor",
                model_file_name=preprocessor_file_name,
            )
        try:
            generator.save(str(trained_model_path))
            if preprocessor_artifact_path is not None and preprocessor is not None:
                preprocessor.save(str(preprocessor_artifact_path))
        except Exception:
            trained_model_path.unlink(missing_ok=True)
            if preprocessor_artifact_path is not None:
                preprocessor_artifact_path.unlink(missing_ok=True)
            raise

        future: Future[list[Path]] = Future()
        request = _AgentSaveRequest(
            project_dirs=tuple(project_dirs),
            trained_model_path=trained_model_path,
            preprocessor_artifact_path=preprocessor_artifact_path,
            training_metrics=self.snapshot(),
            num_demos=num_demos,
            created_at=created_at,
            model_file_name=model_file_name,
            agent_name=agent_name,
            checkpoint_episode=checkpoint_episode,
            preprocessor_file_name=preprocessor_file_name,
            training_settings=training_settings,
            future=future,
        )
        self._save_queue.put(request)
        return future

    def close(self) -> None:
        """Flush queued saves, stop the worker, and surface any background errors."""
        with self._close_lock:
            if self._closed:
                self._raise_if_failed()
                return

            self._closed = True
            self._save_queue.put(_STOP_WORKER)
            self._worker.join()
            self._temp_dir.cleanup()

        self._raise_if_failed()

    def _run_worker(self) -> None:
        while True:
            item = self._save_queue.get()
            try:
                if item is _STOP_WORKER:
                    return

                request = item
                failure = self._get_failure()
                if failure is not None:
                    request.future.set_exception(
                        RuntimeError("Asynchronous training metrics save failed."),
                    )
                    continue

                try:
                    saved_agent_dirs = [
                        create_agent_folder(
                            project_dir,
                            trained_model_path=request.trained_model_path,
                            training_metrics=request.training_metrics,
                            num_demos=request.num_demos,
                            created_at=request.created_at,
                            model_file_name=request.model_file_name,
                            agent_name=request.agent_name,
                            checkpoint_episode=request.checkpoint_episode,
                            preprocessor_artifact_path=request.preprocessor_artifact_path,
                            preprocessor_file_name=request.preprocessor_file_name,
                            training_settings=request.training_settings,
                        )
                        for project_dir in request.project_dirs
                    ]
                except Exception as exc:
                    self._set_failure(exc)
                    request.future.set_exception(exc)
                else:
                    request.future.set_result(saved_agent_dirs)
                    if self._save_callback is not None:
                        self._save_callback(
                            {
                                "type": "agent_saved",
                                "agent_name": request.agent_name,
                                "checkpoint_episode": request.checkpoint_episode,
                                "is_final": request.checkpoint_episode is None,
                            },
                        )
            finally:
                if item is not _STOP_WORKER:
                    item.trained_model_path.unlink(missing_ok=True)
                    if item.preprocessor_artifact_path is not None:
                        item.preprocessor_artifact_path.unlink(missing_ok=True)
                self._save_queue.task_done()

    def _reserve_model_path(self, *, agent_name: str, model_file_name: str) -> Path:
        suffix = Path(model_file_name).suffix or ".bin"
        return self._temp_dir_path / f"{agent_name}_{uuid4().hex}{suffix}"

    def _append_extra_metrics(
        self,
        *,
        previous_episode_count: int,
        extra_metrics: Mapping[str, float],
    ) -> None:
        for metric_name, series in self._metrics.items():
            if metric_name in _BASE_METRIC_NAMES or metric_name in extra_metrics:
                continue
            series.append(float("nan"))

        for metric_name, raw_value in extra_metrics.items():
            if metric_name in _BASE_METRIC_NAMES:
                continue
            if metric_name not in self._metrics:
                self._metrics[metric_name] = [float("nan")] * previous_episode_count
            self._metrics[metric_name].append(float(raw_value))

    def _get_failure(self) -> Exception | None:
        with self._failure_lock:
            return self._worker_failure

    def _set_failure(self, exc: Exception) -> None:
        with self._failure_lock:
            if self._worker_failure is None:
                self._worker_failure = exc

    def _raise_if_failed(self) -> None:
        failure = self._get_failure()
        if failure is not None:
            raise RuntimeError(
                "Asynchronous training metrics save failed.",
            ) from failure

    def _raise_if_closed(self) -> None:
        if self._closed:
            raise RuntimeError("TrainingMetricsSession is already closed.")
