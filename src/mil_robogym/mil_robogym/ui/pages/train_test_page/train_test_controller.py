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

    def __init__(self, view: tk.Frame, app):
        self.view = view
        self.app = app

        self.raw_project = None
        self.project: RoboGymProjectYaml | None = None

        self.trainer: Trainer | None = None
        self.training_settings: dict[str, object] = {}

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
        self.app.show_page("start")

    def navigate_to_project(self, _event=None) -> None:
        """
        Navigate to project page.
        """
        self.app.show_page("view_project", project=self.raw_project)

    def navigate_to_settings(self) -> None:
        """
        Navigate to the training settings page.
        """
        self.app.show_page("training_settings", project=self.raw_project)

    def start_training(self) -> None:
        """
        Start training loop.
        """
        if self.project is None:
            self.view.set_terminal_text("Training unavailable: no project is loaded.")
            return

        self.view.set_training_enabled(False)
        self.view.set_terminal_text(
            "Training agent with saved settings...\n"
            "The history and metrics panels will refresh when it finishes.",
        )
        self.view.flush_ui_updates()

        try:
            self.trainer = Trainer(self.project, **self.training_settings)
            saved_agent_dirs = self.trainer.train()
            latest_agent_name = saved_agent_dirs[0].name if saved_agent_dirs else None
            self.view.refresh_project_artifacts(latest_agent_name)
            self.view.set_terminal_text(
                "Training finished.\n"
                f"Latest saved agent: {latest_agent_name or 'unknown'}",
            )
        except Exception as e:
            traceback.print_exc()
            self.view.set_terminal_text(
                "Training failed.\n" f"{type(e).__name__}: {e}",
            )
        finally:
            self.view.set_training_enabled(True)
