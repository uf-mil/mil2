from __future__ import annotations

import tkinter as tk
from typing import Callable


class ProjectFieldsSection:
    """Render the project, world, and model field controls.

    Args:
        parent: Parent widget that hosts this section.
        world_default: Default world-file string used to initialize the field.
        on_project_name_change: Callback fired when project name changes.
    """

    def __init__(
        self,
        parent: tk.Widget,
        world_default: str,
        on_project_name_change: Callable[[], None],
    ) -> None:
        """Initialize and place all widgets for the fields section."""
        self.project_name_label = tk.Label(
            parent,
            text="Project Name:",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15),
            anchor="w",
        )
        self.project_name_label.grid(
            row=1,
            column=0,
            sticky="w",
            padx=(14, 8),
            pady=5,
        )

        self.project_name_var = tk.StringVar()
        self.project_name_var.trace_add("write", lambda *_: on_project_name_change())
        self.project_name_entry = tk.Entry(
            parent,
            textvariable=self.project_name_var,
            font=("Arial", 15),
        )
        self.project_name_entry.grid(
            row=1,
            column=1,
            columnspan=5,
            sticky="nsew",
            padx=(0, 14),
            pady=5,
            ipady=3,
        )

        self.world_file_label = tk.Label(
            parent,
            text="World File:",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15),
            anchor="w",
        )
        self.world_file_label.grid(
            row=2,
            column=0,
            sticky="w",
            padx=(14, 8),
            pady=5,
        )
        self.world_file_var = tk.StringVar(value=world_default)
        self.world_file_entry = tk.Entry(
            parent,
            textvariable=self.world_file_var,
            font=("Arial", 15),
            state=tk.DISABLED,
        )
        self.world_file_entry.grid(
            row=2,
            column=1,
            columnspan=4,
            sticky="nsew",
            padx=(0, 8),
            pady=5,
            ipady=3,
        )

        self.model_name_label = tk.Label(
            parent,
            text="Model Name:",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15),
            anchor="w",
        )
        self.model_name_label.grid(
            row=3,
            column=0,
            sticky="w",
            padx=(14, 8),
            pady=5,
        )
        self.model_name_var = tk.StringVar(value="sub9")
        self.model_name_entry = tk.Entry(
            parent,
            textvariable=self.model_name_var,
            font=("Arial", 15),
            state=tk.DISABLED,
        )
        self.model_name_entry.grid(
            row=3,
            column=1,
            columnspan=3,
            sticky="nsew",
            padx=(0, 8),
            pady=5,
            ipady=3,
        )
