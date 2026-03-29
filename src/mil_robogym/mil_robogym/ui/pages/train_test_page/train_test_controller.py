import queue
import threading
import tkinter as tk
import traceback
from typing import Any, Mapping

from mil_robogym.data_collection.types import RoboGymProjectYaml
from mil_robogym.vairl.trainer import Trainer
from mil_robogym.vairl.training_settings import normalize_training_settings


class TrainTestViewController:
    """
    Class that handles logic for training and testing agents and updating UI.
    """

    _POLL_INTERVAL_MS = 100

    def __init__(self, view: tk.Frame, app):
        self.view = view
        self.app = app

        self.raw_project = None
        self.project: RoboGymProjectYaml | None = None

        self.trainer: Trainer | None = None
        self.training_settings: dict[str, object] = {}
        self._training_event_queue: queue.Queue[dict[str, object]] = queue.Queue()
        self._training_thread: threading.Thread | None = None
        self._poll_after_id: str | None = None

    def set_context(self, project: Mapping[str, Any] | None = None) -> None:

        self.raw_project = project
        self.project = (
            project.get("robogym_project", {}) if project is not None else None
        )
        raw_training_settings = (
            project.get("robogym_training", {}) if project is not None else {}
        )
        self.training_settings = normalize_training_settings(raw_training_settings)
        self.trainer = None

    def navigate_to_home(self, _event=None) -> None:
        """
        Navigate to home page.
        """
        if self.is_training_running():
            self.view.set_terminal_text("Training is still running.")
            return
        if hasattr(self.view, "persist_ui_state"):
            self.view.persist_ui_state()
        self.app.show_page("start")

    def navigate_to_project(self, _event=None) -> None:
        """
        Navigate to project page.
        """
        if self.is_training_running():
            self.view.set_terminal_text("Training is still running.")
            return
        if hasattr(self.view, "persist_ui_state"):
            self.view.persist_ui_state()
        self.app.show_page("view_project", project=self.raw_project)

    def navigate_to_settings(self) -> None:
        """
        Navigate to the training settings page.
        """
        if self.is_training_running():
            self.view.set_terminal_text("Training is still running.")
            return
        if hasattr(self.view, "persist_ui_state"):
            self.view.persist_ui_state()
        self.app.show_page("training_settings", project=self.raw_project)

    def start_training(self) -> None:
        """
        Start training loop.
        """
        if self.project is None:
            self.view.set_terminal_text("Training unavailable: no project is loaded.")
            return
        if self.is_training_running():
            self.view.set_terminal_text("Training is already running.")
            return

        self.view.set_training_enabled(False)
        self.view.set_terminal_text(
            "Training agent with saved settings...\n"
            "Live metrics will refresh while it runs.",
        )
        self.view.show_live_metrics({})
        self.view.flush_ui_updates()
        self._training_event_queue = queue.Queue()
        self._training_thread = threading.Thread(
            target=self._run_training_worker,
            name="mil_robogym_training_ui_worker",
            daemon=True,
        )
        self._training_thread.start()
        self._schedule_training_poll()

    def is_training_running(self) -> bool:
        return self._training_thread is not None and self._training_thread.is_alive()

    def wait_for_training_completion(self, timeout: float | None = None) -> None:
        if self._training_thread is not None:
            self._training_thread.join(timeout)

    def process_pending_training_events(self) -> None:
        events_to_process: list[dict[str, object]] = []
        latest_metrics_event: dict[str, object] | None = None

        while True:
            try:
                event = self._training_event_queue.get_nowait()
            except queue.Empty:
                break

            if event.get("type") == "metrics_updated":
                latest_metrics_event = event
                continue

            if latest_metrics_event is not None:
                events_to_process.append(latest_metrics_event)
                latest_metrics_event = None
            events_to_process.append(event)

        if latest_metrics_event is not None:
            events_to_process.append(latest_metrics_event)

        for event in events_to_process:
            self._handle_training_event(event)

    def _run_training_worker(self) -> None:
        if self.project is None:
            return

        try:
            self.trainer = Trainer(
                self.project,
                **self.training_settings,
                progress_callback=self._enqueue_training_event,
            )
            saved_agent_dirs = self.trainer.train()
            latest_agent_name = saved_agent_dirs[0].name if saved_agent_dirs else None
            self._enqueue_training_event(
                {
                    "type": "training_finished",
                    "latest_agent_name": latest_agent_name,
                },
            )
        except Exception as e:
            traceback.print_exc()
            self._enqueue_training_event(
                {
                    "type": "training_failed",
                    "error_type": type(e).__name__,
                    "message": str(e),
                },
            )

    def _enqueue_training_event(self, event: dict[str, object]) -> None:
        self._training_event_queue.put(event)

    def _schedule_training_poll(self) -> None:
        if self._poll_after_id is not None:
            return
        self._poll_after_id = self.view.after(
            self._POLL_INTERVAL_MS,
            self._poll_training_events,
        )

    def _poll_training_events(self) -> None:
        self._poll_after_id = None
        self.process_pending_training_events()
        if self.is_training_running() or not self._training_event_queue.empty():
            self._schedule_training_poll()

    def _handle_training_event(self, event: dict[str, object]) -> None:
        event_type = event.get("type")

        if event_type == "metrics_updated":
            episode = int(event.get("episode", 0))
            num_episodes = int(event.get("num_episodes", 0))
            metrics = event.get("metrics", {})
            if isinstance(metrics, dict):
                self.view.show_live_metrics(metrics)
            self.view.set_terminal_text(
                "Training agent with saved settings...\n"
                f"Episode {episode}/{num_episodes}",
            )
            return

        if event_type == "agent_saved":
            agent_name = event.get("agent_name")
            if isinstance(agent_name, str):
                self.view.refresh_history(agent_name)
            return

        if event_type == "training_finished":
            latest_agent_name = event.get("latest_agent_name")
            if isinstance(latest_agent_name, str):
                self.view.refresh_project_artifacts(latest_agent_name)
            else:
                self.view.refresh_project_artifacts(None)
            self.view.set_terminal_text(
                "Training finished.\n"
                f"Latest saved agent: {latest_agent_name or 'unknown'}",
            )
            self.view.set_training_enabled(True)
            return

        if event_type == "training_failed":
            error_type = event.get("error_type", "RuntimeError")
            message = event.get("message", "unknown error")
            self.view.set_terminal_text(
                "Training failed.\n" f"{error_type}: {message}",
            )
            self.view.set_training_enabled(True)
