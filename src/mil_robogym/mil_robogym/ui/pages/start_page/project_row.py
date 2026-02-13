import tkinter as tk
from collections.abc import Callable
from typing import Any


class ProjectRow(tk.Frame):
    """
    A clickable, button-like row widget with left and right-aligned text.

    :param parent: Parent Tkinter widget that will contain this row.
    :param left_text: Text displayed on the left side of the row.
    :param right_text: Text displayed on the right side of the row.
    :param command: Optional callback invoked when the row is clicked.
    """

    def __init__(
        self,
        parent: tk.Widget,
        left_text: str,
        right_text: str,
        command: Callable[[], Any] | None = None,
    ) -> None:

        super().__init__(
            parent,
            bg="#E6E6E6",
            bd=1,
            relief="solid",
            highlightthickness=0,
        )

        inner: tk.Frame = tk.Frame(self, bg="#E6E6E6")
        inner.grid(row=0, column=0, sticky="nsew", padx=10, pady=6)

        inner.grid_columnconfigure(0, weight=1)
        inner.grid_columnconfigure(1, weight=0)

        left: tk.Label = tk.Label(
            inner,
            text=left_text,
            bg="#E6E6E6",
            fg="black",
            anchor="w",
        )
        right: tk.Label = tk.Label(
            inner,
            text=right_text,
            bg="#E6E6E6",
            fg="black",
            anchor="e",
        )

        left.grid(row=0, column=0, sticky="w")
        right.grid(row=0, column=1, sticky="e", padx=(20, 0))

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        def on_click(_event: tk.Event | None = None) -> None:
            """Handle left-click events by invoking ``command`` if callable."""
            if callable(command):
                command()

        if command is not None:
            self.configure(cursor="hand2")
            for w in (self, inner, left, right):
                w.bind("<Button-1>", on_click)

        def on_enter(_event: tk.Event | None = None) -> None:
            """Apply hover styling when the pointer enters the widget."""
            if command is None:
                return
            for w in (self, inner, left, right):
                w.configure(bg="#DCDCDC")

        def on_leave(_event: tk.Event | None = None) -> None:
            """Remove hover styling when the pointer leaves the widget."""
            if command is None:
                return
            for w in (self, inner, left, right):
                w.configure(bg="#E6E6E6")

        if command is not None:
            for w in (self, inner, left, right):
                w.bind("<Enter>", on_enter)
                w.bind("<Leave>", on_leave)
