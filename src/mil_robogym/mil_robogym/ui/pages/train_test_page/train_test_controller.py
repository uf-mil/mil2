import tkinter as tk
import traceback
from typing import Any, Mapping

from mil_robogym.data_collection.types import RoboGymProjectYaml
from mil_robogym.vairl.trainer import Trainer

UI_TEST_NUM_EPISODES = 1
UI_TEST_ROLLOUT_STEPS = 8
UI_TEST_MAX_STEP_COUNT = 3


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

    def set_context(self, project: Mapping[str, Any] | None = None) -> None:

        self.raw_project = project
        self.project = (
            project.get("robogym_project", {}) if project is not None else None
        )

        self.trainer = Trainer(self.project) if self.project is not None else None

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

    def start_training(self) -> None:
        """
        Start training loop.
        """
        if self.trainer is None:
            self.view.set_terminal_text("Training unavailable: no project is loaded.")
            return

        self.view.set_training_enabled(False)
        self.view.set_terminal_text(
            "Training agent for 1 very short episode...\n"
            "The history and metrics panels will refresh when it finishes.",
        )
        self.view.flush_ui_updates()

        self.trainer.num_episodes = UI_TEST_NUM_EPISODES
        self.trainer.rollout_steps = min(
            self.trainer.rollout_steps,
            UI_TEST_ROLLOUT_STEPS,
        )
        current_max_step_count = self.trainer.max_step_count
        if current_max_step_count is None:
            self.trainer.max_step_count = UI_TEST_MAX_STEP_COUNT
        else:
            self.trainer.max_step_count = min(
                current_max_step_count,
                UI_TEST_MAX_STEP_COUNT,
            )
        try:
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
