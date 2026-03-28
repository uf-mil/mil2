from __future__ import annotations

import tkinter as tk


class TerminalSection:
    """Terminal output panel for train/test actions."""

    def __init__(self, parent: tk.Widget, terminal_text: str) -> None:
        self.container = tk.Frame(parent, bg="#DADADA")
        self.container.grid(
            row=2,
            column=2,
            columnspan=4,
            sticky="nsew",
            padx=(8, 14),
            pady=(0, 8),
        )
        self.container.grid_columnconfigure(0, weight=1)
        self.container.grid_rowconfigure(1, weight=1)

        self.title_label = tk.Label(
            self.container,
            text="Terminal",
            bg="#DADADA",
            fg="black",
            font=("Arial", 18, "bold"),
            anchor="w",
        )
        self.title_label.grid(row=0, column=0, sticky="w", pady=(0, 4))

        self.output_label = tk.Label(
            self.container,
            text=terminal_text,
            bg="#F2F2F2",
            fg="#333333",
            font=("Arial", 10),
            anchor="center",
            justify="center",
            relief="solid",
            bd=1,
            padx=8,
            pady=8,
        )
        self.output_label.grid(row=1, column=0, sticky="nsew")

    def set_text(self, terminal_text: str) -> None:
        """Update the terminal panel text."""
        self.output_label.configure(text=terminal_text)
