from __future__ import annotations

import tkinter as tk
from tkinter import messagebox
from typing import Any, Mapping

from mil_robogym.data_collection.filesystem import update_project_training_settings
from mil_robogym.data_collection.get_all_project_config import get_all_project_config
from mil_robogym.data_collection.types import RoboGymProjectYaml, RoboGymTrainingYaml
from mil_robogym.vairl.training_settings import (
    get_default_training_settings,
    normalize_training_settings,
)

FIELD_SPECS: list[tuple[str, str, str]] = [
    ("num_episodes", "Num Episodes", "Total training episodes to run."),
    ("rollout_steps", "Rollout Steps", "Timesteps collected per generator update."),
    (
        "save_every",
        "Save Every",
        "Checkpoint cadence in episodes. Use 0 for final-only.",
    ),
    ("seed", "Seed", "Random seed used for training."),
    ("max_step_count", "Max Step Count", "Leave blank to auto-derive from demos."),
    (
        "generator_learning_rate",
        "Generator Learning Rate",
        "Learning rate for the TRPO generator.",
    ),
    (
        "discriminator_learning_rate",
        "Discriminator Learning Rate",
        "Learning rate for the discriminator/reward net update.",
    ),
    ("gamma", "Gamma", "Discount factor. Must be > 0 and <= 1."),
    ("beta_step_size", "Beta Step Size", "Constraint beta adjustment step size."),
    ("i_c", "I_c", "Information bottleneck target."),
    ("z_size", "Z Size", "Latent bottleneck size."),
    ("e_hidden_size", "E Hidden Size", "Reward-net encoder hidden size."),
    (
        "expert_noise_std",
        "Expert Noise Std",
        "Noise applied while loading expert trajectories.",
    ),
]


class TrainingSettingsPage(tk.Frame):
    """Page for editing persisted training settings for a project."""

    def __init__(self, parent: tk.Widget, controller: Any | None = None) -> None:
        super().__init__(parent, bg="#DADADA")
        self.controller = controller
        self.project: Mapping[str, Any] | None = None
        self.project_yaml: RoboGymProjectYaml | None = None
        self.project_name = "Project"
        self._vars: dict[str, tk.StringVar] = {}

        self._build_header()
        self._build_form()
        self._build_actions()

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

    def set_context(
        self,
        project: Mapping[str, Any] | None = None,
        **_kwargs: Any,
    ) -> None:
        self.project = project
        self.project_yaml = (
            project.get("robogym_project", {}) if project is not None else None
        )
        self.project_name = (
            str(self.project_yaml.get("name", "Project"))
            if self.project_yaml is not None
            else "Project"
        )

        self.project_label.configure(text=f"{self.project_name} >")
        settings = normalize_training_settings(
            project.get("robogym_training") if project is not None else None,
        )
        self._populate_form(settings)

    def _build_header(self) -> None:
        self.header = tk.Frame(self, bg="#DADADA")
        self.header.grid(row=0, column=0, sticky="ew", padx=14, pady=(14, 8))
        self.header.grid_columnconfigure(0, weight=1)

        breadcrumb = tk.Frame(self.header, bg="#DADADA")
        breadcrumb.grid(row=0, column=0, sticky="w")

        self.home_label = tk.Label(
            breadcrumb,
            text="MIL RoboGYM >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            cursor="hand2",
        )
        self.home_label.pack(side="left")
        self.home_label.bind("<Button-1>", self._navigate_home)

        self.project_label = tk.Label(
            breadcrumb,
            text="Project >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            cursor="hand2",
        )
        self.project_label.pack(side="left", padx=(6, 0))
        self.project_label.bind("<Button-1>", self._navigate_project)

        self.train_test_label = tk.Label(
            breadcrumb,
            text="Train/Test >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            cursor="hand2",
        )
        self.train_test_label.pack(side="left", padx=(6, 0))
        self.train_test_label.bind("<Button-1>", self._navigate_train_test)

        self.page_label = tk.Label(
            breadcrumb,
            text="Settings",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
        )
        self.page_label.pack(side="left", padx=(6, 0))

    def _build_form(self) -> None:
        self.form_shell = tk.Frame(
            self,
            bg="#F2F2F2",
            relief="solid",
            bd=1,
            padx=16,
            pady=16,
        )
        self.form_shell.grid(row=1, column=0, sticky="nsew", padx=14, pady=(0, 8))
        for col in range(4):
            self.form_shell.grid_columnconfigure(col, weight=1)

        intro = tk.Label(
            self.form_shell,
            text=(
                "Adjust the training settings used for new agents in this project. "
                "Saved values become the defaults the next time you open Settings."
            ),
            bg="#F2F2F2",
            fg="#333333",
            font=("Arial", 11),
            anchor="w",
            justify="left",
            wraplength=860,
        )
        intro.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 14))

        for index, (field_name, label_text, helper_text) in enumerate(FIELD_SPECS):
            column_group = index % 2
            row_group = index // 2
            row = row_group + 1

            field_card = tk.Frame(
                self.form_shell,
                bg="#F2F2F2",
                padx=0,
                pady=0,
            )
            field_card.grid(
                row=row,
                column=column_group * 2,
                columnspan=2,
                sticky="nsew",
                padx=(0, 12),
                pady=(4, 8),
            )
            field_card.grid_columnconfigure(0, weight=1)

            label = tk.Label(
                field_card,
                text=label_text,
                bg="#F2F2F2",
                fg="black",
                font=("Arial", 11, "bold"),
                anchor="w",
            )
            label.grid(row=0, column=0, sticky="w")

            helper = tk.Label(
                field_card,
                text=helper_text,
                bg="#F2F2F2",
                fg="#555555",
                font=("Arial", 9),
                anchor="w",
                justify="left",
                wraplength=360,
            )
            helper.grid(row=1, column=0, sticky="ew", pady=(2, 6))

            var = tk.StringVar()
            self._vars[field_name] = var
            entry = tk.Entry(
                field_card,
                textvariable=var,
                font=("Arial", 11),
                relief="solid",
                bd=1,
            )
            entry.grid(row=2, column=0, sticky="ew")

    def _build_actions(self) -> None:
        self.actions = tk.Frame(self, bg="#DADADA")
        self.actions.grid(row=2, column=0, sticky="ew", padx=14, pady=(0, 14))
        self.actions.grid_columnconfigure(0, weight=1)
        self.actions.grid_columnconfigure(1, weight=0)
        self.actions.grid_columnconfigure(2, weight=0)

        self.reset_button = tk.Button(
            self.actions,
            text="Restore Defaults",
            command=self._restore_defaults,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 11),
            padx=12,
            pady=4,
            cursor="hand2",
        )
        self.reset_button.grid(row=0, column=0, sticky="w")

        self.cancel_button = tk.Button(
            self.actions,
            text="Back",
            command=self._return_to_train_test,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 11),
            padx=12,
            pady=4,
            cursor="hand2",
        )
        self.cancel_button.grid(row=0, column=1, sticky="e", padx=(0, 8))

        self.save_button = tk.Button(
            self.actions,
            text="Save Settings",
            command=self._save_settings,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 11),
            padx=12,
            pady=4,
            cursor="hand2",
        )
        self.save_button.grid(row=0, column=2, sticky="e")

    def _populate_form(self, settings: RoboGymTrainingYaml) -> None:
        for field_name, _, _ in FIELD_SPECS:
            value = settings[field_name]
            self._vars[field_name].set("" if value is None else str(value))

    def _restore_defaults(self) -> None:
        self._populate_form(get_default_training_settings())

    def _collect_form_values(self) -> RoboGymTrainingYaml:
        raw_settings = {field_name: var.get() for field_name, var in self._vars.items()}
        return normalize_training_settings(raw_settings)

    def _save_settings(self) -> None:
        if self.project_yaml is None:
            messagebox.showerror("Save Settings", "No project is loaded.")
            return

        try:
            normalized_settings = self._collect_form_values()
            update_project_training_settings(self.project_yaml, normalized_settings)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            messagebox.showerror("Save Settings", f"Failed to save settings:\n{exc}")
            return

        self._return_to_train_test()

    def _return_to_train_test(self) -> None:
        if self.controller is None:
            return
        self.controller.show_page(
            "train_test",
            project=self._load_project_payload() or self.project,
        )

    def _navigate_home(self, _event: tk.Event | None = None) -> None:
        if self.controller is not None:
            self.controller.show_page("start")

    def _navigate_project(self, _event: tk.Event | None = None) -> None:
        if self.controller is not None:
            self.controller.show_page(
                "view_project",
                project=self._load_project_payload() or self.project,
            )

    def _navigate_train_test(self, _event: tk.Event | None = None) -> None:
        self._return_to_train_test()

    def _load_project_payload(self) -> Mapping[str, Any] | None:
        if not self.project_name:
            return None
        for project in get_all_project_config():
            if project.get("robogym_project", {}).get("name") == self.project_name:
                return project
        return None
