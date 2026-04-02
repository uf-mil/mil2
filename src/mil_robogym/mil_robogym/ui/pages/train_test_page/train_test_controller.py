import queue
import threading
import tkinter as tk
import traceback
from typing import Any, Mapping

from mil_robogym.data_collection.delete_saved_agent import (
    delete_saved_agent_artifacts,
)
from mil_robogym.data_collection.load_saved_agent import (
    LoadedAgent,
    load_saved_agent_model,
)
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
        self.loaded_agent: LoadedAgent | None = None
        self.training_settings: dict[str, object] = {}
        self._training_event_queue: queue.Queue[dict[str, object]] = queue.Queue()
        self._training_thread: threading.Thread | None = None
        self._poll_after_id: str | None = None
        self._stop_requested = threading.Event()
        self._abort_requested = threading.Event()
        self._latest_saved_agent_name: str | None = None

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
        self.loaded_agent = None

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
        self.loaded_agent = None
        self.view.show_live_metrics({})
        self.view.flush_ui_updates()
        self._stop_requested.clear()
        self._abort_requested.clear()
        self._latest_saved_agent_name = None
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

    def stop_training(self) -> None:
        """Request that the current training run stop at the next safe boundary."""
        if not self.is_training_running():
            self.view.set_terminal_text("Training is not currently running.")
            return
        if self._abort_requested.is_set():
            self.view.set_terminal_text("Training abort is already in progress.")
            return

        self._stop_requested.set()
        if self.trainer is not None:
            self.trainer.request_stop()
        self.view.set_terminal_text(
            "Stopping training...\n" "Waiting for the current episode to finish.",
        )

    def abort_training(self) -> None:
        """Abort training as quickly as the current worker state allows."""
        if not self.is_training_running():
            self.view.set_terminal_text("Training is not currently running.")
            return

        self._abort_requested.set()
        self._stop_requested.set()
        if self.trainer is not None:
            self.trainer.request_abort()
        self.view.set_terminal_text(
            "Aborting training...\n"
            "Unsaved progress from the current episode will be discarded.",
        )

    def wait_for_training_completion(self, timeout: float | None = None) -> None:
        if self._training_thread is not None:
            self._training_thread.join(timeout)

    def load_selected_agent(self) -> LoadedAgent | None:
        """Load the currently selected saved model as a callable agent."""
        if self.is_training_running():
            self.view.set_terminal_text(
                "Cannot load a saved model while training runs.",
            )
            return None
        if self.raw_project is None:
            self.view.set_terminal_text("Load unavailable: no project is loaded.")
            return None

        agent_name = getattr(self.view, "selected_agent_name", None)
        if not isinstance(agent_name, str) or not agent_name.strip():
            self.view.set_terminal_text("Load unavailable: no saved model is selected.")
            return None

        try:
            loaded_agent = load_saved_agent_model(self.raw_project, agent_name)
        except (FileNotFoundError, ValueError) as e:
            self.loaded_agent = None
            self.view.set_terminal_text(
                "Failed to load saved model.\n" f"{type(e).__name__}: {e}",
            )
            return None

        self.loaded_agent = loaded_agent
        agent_kind = (
            f"checkpoint episode {loaded_agent.handle.checkpoint_episode}"
            if loaded_agent.handle.checkpoint_episode is not None
            else "final model"
        )
        self.view.set_terminal_text(
            "Loaded saved model.\n"
            f"{loaded_agent.handle.agent_name} | {agent_kind} | "
            f"in {loaded_agent.input_size} -> out {loaded_agent.output_size}",
        )
        return loaded_agent

    def clear_loaded_agent(self) -> None:
        """Discard any previously loaded saved-model handle."""
        self.loaded_agent = None

    def delete_saved_agent(self, agent_name: str) -> bool:
        """Delete one saved model and clear stale in-memory handles."""
        if self.is_training_running():
            self.view.set_terminal_text(
                "Cannot delete saved models while training runs.",
            )
            return False
        if self.raw_project is None:
            self.view.set_terminal_text("Delete unavailable: no project is loaded.")
            return False

        try:
            deleted_paths = delete_saved_agent_artifacts(self.raw_project, agent_name)
        except (FileNotFoundError, ValueError, OSError) as e:
            self.view.set_terminal_text(
                "Failed to delete saved model.\n" f"{type(e).__name__}: {e}",
            )
            return False

        if (
            self.loaded_agent is not None
            and self.loaded_agent.handle.agent_name == agent_name
        ):
            self.loaded_agent = None
        if self._latest_saved_agent_name == agent_name:
            self._latest_saved_agent_name = None

        deleted_count = len(deleted_paths)
        copy_label = "copy" if deleted_count == 1 else "copies"
        self.view.set_terminal_text(
            "Deleted saved model.\n"
            f"{agent_name} | removed {deleted_count} {copy_label}",
        )
        return True

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
            if self._abort_requested.is_set():
                self.trainer.request_abort()
            elif self._stop_requested.is_set():
                self.trainer.request_stop()
            saved_agent_dirs = self.trainer.train()
            latest_agent_name = saved_agent_dirs[0].name if saved_agent_dirs else None
            self._enqueue_training_event(
                {
                    "type": "training_finished",
                    "latest_agent_name": latest_agent_name,
                    "stopped": self.trainer.was_stopped(),
                    "aborted": self.trainer.was_aborted(),
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
            if not self._stop_requested.is_set():
                self.view.set_terminal_text(
                    "Training agent with saved settings...\n"
                    f"Episode {episode}/{num_episodes}",
                )
            return

        if event_type == "agent_saved":
            agent_name = event.get("agent_name")
            if isinstance(agent_name, str):
                self._latest_saved_agent_name = agent_name
                self.view.refresh_history(agent_name)
            return

        if event_type == "training_finished":
            latest_agent_name = event.get("latest_agent_name")
            resolved_agent_name = (
                latest_agent_name
                if isinstance(latest_agent_name, str)
                else self._latest_saved_agent_name
            )
            if isinstance(resolved_agent_name, str):
                self.view.refresh_project_artifacts(resolved_agent_name)
            else:
                self.view.refresh_project_artifacts(None)
            if bool(event.get("aborted")):
                self.view.set_terminal_text(
                    "Training aborted.\n"
                    f"Latest saved agent: {resolved_agent_name or 'none'}",
                )
            elif bool(event.get("stopped")):
                self.view.set_terminal_text(
                    "Training stopped.\n"
                    f"Latest saved agent: {resolved_agent_name or 'unknown'}",
                )
            else:
                self.view.set_terminal_text(
                    "Training finished.\n"
                    f"Latest saved agent: {resolved_agent_name or 'unknown'}",
                )
            self.view.set_training_enabled(True)
            self._stop_requested.clear()
            self._abort_requested.clear()
            return

        if event_type == "training_failed":
            error_type = event.get("error_type", "RuntimeError")
            message = event.get("message", "unknown error")
            self.view.set_terminal_text(
                "Training failed.\n" f"{error_type}: {message}",
            )
            self.view.set_training_enabled(True)
            self._stop_requested.clear()
            self._abort_requested.clear()
