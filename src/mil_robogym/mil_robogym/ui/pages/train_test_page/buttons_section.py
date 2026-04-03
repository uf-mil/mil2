from __future__ import annotations

import tkinter as tk
from typing import Callable


class ButtonsSection:
    """Bottom action buttons for training and testing."""

    def __init__(
        self,
        parent: tk.Widget,
        on_train_click: Callable[[], None],
        on_stop_click: Callable[[], None],
        on_abort_click: Callable[[], None],
        on_test_click: Callable[[], None],
    ) -> None:
        self._on_train_click = on_train_click
        self._on_stop_click = on_stop_click
        self._on_abort_click = on_abort_click
        self._training_running = False

        self.container = tk.Frame(parent, bg="#DADADA")
        self.container.grid(
            row=3,
            column=2,
            columnspan=4,
            sticky="ew",
            padx=(8, 14),
            pady=(0, 14),
        )
        self.container.grid_columnconfigure(0, weight=1)
        self.container.grid_columnconfigure(1, weight=0)

        self.train_button = tk.Button(
            self.container,
            text="Train New Agent",
            command=self._handle_train_button_click,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 10),
            padx=10,
            pady=4,
            cursor="hand2",
        )
        self.train_button.grid(row=0, column=0, sticky="ew", padx=(0, 8))

        self.test_button = tk.Button(
            self.container,
            text="Test Selected Agent",
            command=on_test_click,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 10),
            padx=10,
            pady=4,
            cursor="hand2",
        )
        self.test_button.grid(row=0, column=1, sticky="e", padx=(8, 0))

        self.abort_button = tk.Button(
            self.container,
            text="Abort",
            command=self._on_abort_click,
            bg="#E6C8C3",
            activebackground="#DDB4AE",
            fg="#3C0903",
            relief="solid",
            bd=1,
            font=("Arial", 9),
            padx=8,
            pady=2,
            cursor="hand2",
        )
        self.abort_button.grid(row=0, column=1, sticky="e", padx=(8, 0))
        self.abort_button.grid_remove()

    def set_training_enabled(self, enabled: bool) -> None:
        """Update button state to match whether training is currently running."""
        self._training_running = not enabled

        if self._training_running:
            self.test_button.grid_remove()
            self.abort_button.grid()
            self.abort_button.configure(state="normal")
            self.train_button.configure(
                text="Stop Training",
                command=self._handle_train_button_click,
                state="normal",
                bg="#F2D7D4",
                activebackground="#EBC3BF",
            )
            self.test_button.configure(state="disabled")
            return

        self.train_button.configure(
            text="Train New Agent",
            command=self._handle_train_button_click,
            state="normal",
            bg="#ECECEC",
            activebackground="#DFDFDF",
        )
        self.abort_button.configure(state="disabled")
        self.abort_button.grid_remove()
        self.test_button.grid()
        self.test_button.configure(state="normal")

    def _handle_train_button_click(self) -> None:
        if self._training_running:
            self._on_stop_click()
            return
        self._on_train_click()
