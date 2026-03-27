from __future__ import annotations

import tkinter as tk
from typing import Callable


class HeaderSection:
    """Header area for the Train/Test page."""

    def __init__(
        self,
        parent: tk.Widget,
        on_home_click: Callable[[object], None],
        on_project_click: Callable[[object], None],
        on_settings_click: Callable[[], None],
    ) -> None:
        """Render breadcrumb, last-session text, and modify button."""
        self.container = tk.Frame(parent, bg="#DADADA")
        self.container.grid(
            row=0,
            column=0,
            columnspan=6,
            sticky="ew",
            padx=14,
            pady=(14, 8),
        )
        self.container.grid_columnconfigure(0, weight=1)
        self.container.grid_columnconfigure(1, weight=0)

        left = tk.Frame(self.container, bg="#DADADA")
        left.grid(row=0, column=0, sticky="w")

        self.home_label = tk.Label(
            left,
            text="MIL RoboGYM >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            cursor="hand2",
        )
        self.home_label.pack(side="left")
        self.home_label.bind("<Button-1>", on_home_click)

        self.project_label = tk.Label(
            left,
            text="Project >",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
            cursor="hand2",
        )
        self.project_label.pack(side="left", padx=(6, 0))
        self.project_label.bind("<Button-1>", on_project_click)

        self.page_label = tk.Label(
            left,
            text="Train/Test",
            bg="#DADADA",
            fg="black",
            font=("Arial", 20, "bold"),
        )
        self.page_label.pack(side="left", padx=(6, 0))

        self.last_training_label = tk.Label(
            self.container,
            text="Last training session: N/A",
            bg="#DADADA",
            fg="black",
            font=("Arial", 10),
            anchor="w",
        )
        self.last_training_label.grid(row=1, column=0, sticky="w", pady=(4, 0))

        self.actions_frame = tk.Frame(self.container, bg="#DADADA")
        self.actions_frame.grid(row=0, column=1, rowspan=2, sticky="ne")
        self.actions_frame.grid_columnconfigure(0, weight=0)
        self.actions_frame.grid_columnconfigure(1, weight=0)

        self.settings_button = tk.Button(
            self.actions_frame,
            text="Settings",
            command=on_settings_click,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 10),
            padx=10,
            pady=2,
            cursor="hand2",
        )
        self.settings_button.grid(row=0, column=0, sticky="e", padx=(0, 8))

        self.modify_layers_button = tk.Button(
            self.actions_frame,
            text="Modify Agent Layers",
            command=self._on_modify_layers_click,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 10),
            padx=10,
            pady=2,
            cursor="hand2",
        )
        self.modify_layers_button.grid(row=0, column=1, sticky="e")

    def set_project_name(self, project_name: str) -> None:
        """Update breadcrumb project text."""
        self.project_label.configure(text=f"{project_name} >")

    def set_last_training_session(self, agent_name: str | None) -> None:
        """Update the last-training text from selected/latest agent."""
        if agent_name:
            self.last_training_label.configure(
                text=f"Last training session: {agent_name}",
            )
            return
        self.last_training_label.configure(text="Last training session: N/A")

    def _on_modify_layers_click(self) -> None:
        """Placeholder callback for modifying layer settings."""
        print("clicked")
