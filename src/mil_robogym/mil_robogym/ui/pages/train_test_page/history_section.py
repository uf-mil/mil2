from __future__ import annotations

import tkinter as tk
from pathlib import Path
from typing import Callable

from mil_robogym.data_collection.get_all_agent_config import get_all_agent_config


class HistorySection:
    """History list for saved agents."""

    def __init__(
        self,
        parent: tk.Widget,
        on_agent_click: Callable[[str], None],
        on_delete_click: Callable[[str], None],
    ) -> None:
        self._on_agent_click = on_agent_click
        self._on_delete_click = on_delete_click

        self.container = tk.Frame(parent, bg="#DADADA")
        self.container.grid(
            row=1,
            column=0,
            rowspan=2,
            columnspan=2,
            sticky="nsew",
            padx=(14, 8),
            pady=(0, 8),
        )
        self.container.grid_columnconfigure(0, weight=1)
        self.container.grid_rowconfigure(1, weight=1)

        self.title_label = tk.Label(
            self.container,
            text="History",
            bg="#DADADA",
            fg="black",
            font=("Arial", 18, "bold"),
            anchor="w",
        )
        self.title_label.grid(row=0, column=0, sticky="w", pady=(0, 4))

        self.rows_frame = tk.Frame(self.container, bg="#DADADA")
        self.rows_shell = tk.Frame(
            self.container,
            bg="#B9B9B9",
            relief="solid",
            bd=1,
            padx=4,
            pady=4,
        )
        self.rows_shell.grid(row=1, column=0, sticky="nsew")
        self.rows_shell.grid_columnconfigure(0, weight=1)
        self.rows_shell.grid_rowconfigure(0, weight=1)

        self.rows_frame = tk.Frame(self.rows_shell, bg="#B9B9B9")
        self.rows_frame.grid(row=0, column=0, sticky="nsew")
        self.rows_frame.grid_columnconfigure(0, weight=1)

    def load_agents(self, project_dir: Path) -> list[str]:
        """Load and render all agent rows for a project."""
        for child in self.rows_frame.winfo_children():
            child.destroy()

        agent_config = get_all_agent_config(project_dir)
        if not agent_config:
            empty = tk.Label(
                self.rows_frame,
                text="No agents found.",
                bg="#DADADA",
                fg="#444444",
                font=("Arial", 10),
                anchor="w",
            )
            empty.grid(row=0, column=0, sticky="w", pady=4)
            return []

        agent_names = sorted(agent_config.keys(), reverse=True)
        for row_index, agent_name in enumerate(agent_names):
            self._build_agent_row(
                row_index=row_index,
                agent_name=agent_name,
                num_demos=agent_config[agent_name]["num_demos"],
            )
        return agent_names

    def _build_agent_row(self, row_index: int, agent_name: str, num_demos: int) -> None:
        """Create one clickable history row."""
        row = tk.Frame(
            self.rows_frame,
            bg="#ECECEC",
            relief="solid",
            bd=1,
            cursor="hand2",
        )
        row.grid(row=row_index, column=0, sticky="ew", pady=(0, 4))
        row.grid_columnconfigure(0, weight=1)
        row.grid_columnconfigure(1, weight=0)
        row.grid_columnconfigure(2, weight=0)

        agent_label = tk.Label(
            row,
            text=agent_name,
            bg="#ECECEC",
            fg="black",
            font=("Arial", 10),
            anchor="w",
            padx=6,
            pady=4,
            cursor="hand2",
        )
        agent_label.grid(row=0, column=0, sticky="w")

        demos_label = tk.Label(
            row,
            text=f"{num_demos} Demos",
            bg="#ECECEC",
            fg="black",
            font=("Arial", 10),
            anchor="w",
            padx=6,
            pady=4,
            cursor="hand2",
        )
        demos_label.grid(row=0, column=1, sticky="w")

        delete_button = tk.Button(
            row,
            text="Delete",
            command=lambda name=agent_name: self._on_delete_clicked(name),
            bg="#F1DDDA",
            activebackground="#E8CAC5",
            fg="#4A120A",
            relief="flat",
            bd=0,
            font=("Arial", 9, "bold"),
            padx=6,
            pady=2,
            cursor="hand2",
        )
        delete_button.grid(row=0, column=2, sticky="e", padx=(6, 2))

        self._bind_row_click(row, agent_label, demos_label, agent_name)

    def _bind_row_click(
        self,
        row: tk.Frame,
        agent_label: tk.Label,
        demos_label: tk.Label,
        agent_name: str,
    ) -> None:
        """Bind click and hover behavior for row widgets."""

        def on_click(_event: tk.Event | None = None) -> None:
            self._on_agent_click(agent_name)

        def on_enter(_event: tk.Event | None = None) -> None:
            for widget in (row, agent_label, demos_label):
                widget.configure(bg="#DCDCDC")

        def on_leave(_event: tk.Event | None = None) -> None:
            for widget in (row, agent_label, demos_label):
                widget.configure(bg="#ECECEC")

        for widget in (row, agent_label, demos_label):
            widget.bind("<Button-1>", on_click)
            widget.bind("<Enter>", on_enter)
            widget.bind("<Leave>", on_leave)

    def _on_delete_clicked(self, agent_name: str) -> None:
        """Handle row-action delete behavior."""
        self._on_delete_click(agent_name)
