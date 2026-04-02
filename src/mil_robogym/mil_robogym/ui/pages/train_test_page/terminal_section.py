from __future__ import annotations

import tkinter as tk
from tkinter import scrolledtext


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

        self.output_text = scrolledtext.ScrolledText(
            self.container,
            bg="#F2F2F2",
            fg="#333333",
            font=("Arial", 10),
            relief="solid",
            bd=1,
            wrap="word",
            height=10,
        )
        self.output_text.grid(row=1, column=0, sticky="nsew")
        self.output_text.insert("1.0", terminal_text)
        self.output_text.configure(state="disabled")

    def set_text(self, terminal_text: str) -> None:
        """Replace terminal output text."""
        self.output_text.configure(state="normal")
        self.output_text.delete("1.0", tk.END)
        self.output_text.insert(tk.END, terminal_text)
        self.output_text.see(tk.END)
        self.output_text.configure(state="disabled")

    def append_text(self, terminal_text: str) -> None:
        """Append new text and keep the view scrolled to the newest line."""
        if not terminal_text:
            return
        self.output_text.configure(state="normal")
        self.output_text.insert(tk.END, terminal_text)
        self.output_text.see(tk.END)
        self.output_text.configure(state="disabled")
