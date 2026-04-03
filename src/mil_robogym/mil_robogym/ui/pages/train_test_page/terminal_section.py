from __future__ import annotations

import tkinter as tk
from tkinter import scrolledtext


class TerminalSection:
    """Terminal output panel for train/test actions."""

    _FILTERED_TERMINAL_PHRASES = [
        "Pose set successfully",
    ]

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
        self._terminal_line_buffer = ""
        self.output_text.insert("1.0", self._filter_terminal_text(terminal_text))
        self.output_text.configure(state="disabled")

    def set_text(self, terminal_text: str) -> None:
        """Replace terminal output text."""
        self._terminal_line_buffer = ""
        filtered_text = self._filter_terminal_text(terminal_text)
        self.output_text.configure(state="normal")
        self.output_text.delete("1.0", tk.END)
        self.output_text.insert(tk.END, filtered_text)
        self.output_text.see(tk.END)
        self.output_text.configure(state="disabled")

    def append_text(self, terminal_text: str) -> None:
        """Append new text and keep the view scrolled to the newest line."""
        if not terminal_text:
            return
        combined_text = f"{self._terminal_line_buffer}{terminal_text}"
        lines = combined_text.splitlines(keepends=True)
        if lines and not lines[-1].endswith(("\n", "\r")):
            self._terminal_line_buffer = lines.pop()
        else:
            self._terminal_line_buffer = ""
        filtered_text = self._filter_terminal_text("".join(lines))
        if not filtered_text:
            return
        self.output_text.configure(state="normal")
        self.output_text.insert(tk.END, filtered_text)
        self.output_text.see(tk.END)
        self.output_text.configure(state="disabled")

    def _filter_terminal_text(self, terminal_text: str) -> str:
        filtered_lines: list[str] = []
        for line in terminal_text.splitlines(keepends=True):
            if not self._should_filter_line(line):
                filtered_lines.append(line)
        return "".join(filtered_lines)

    def _should_filter_line(self, line: str) -> bool:
        normalized_line = line.lower()
        return any(
            phrase.lower() in normalized_line
            for phrase in self._FILTERED_TERMINAL_PHRASES
        )
