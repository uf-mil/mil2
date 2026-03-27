import tkinter as tk
from typing import Any, Mapping

from mil_robogym.data_collection.types import RoboGymProjectYaml
from mil_robogym.vairl.trainer import Trainer


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
        self.project = project.get("robogym_project", {})

        self.trainer = Trainer(self.project)

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
        self.trainer.num_episodes = 100
        self.trainer.train()
