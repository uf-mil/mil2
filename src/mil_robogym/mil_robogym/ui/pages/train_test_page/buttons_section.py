from __future__ import annotations

import tkinter as tk
from typing import Callable


class ButtonsSection:
    """Bottom action buttons for training and testing."""

    def __init__(
        self,
        parent: tk.Widget,
        on_train_click: Callable[[], None],
        on_test_click: Callable[[], None],
    ) -> None:
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
        self.container.grid_columnconfigure(1, weight=1)

        self.train_button = tk.Button(
            self.container,
            text="Train New Agent",
            command=on_train_click,
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
        self.test_button.grid(row=0, column=1, sticky="ew", padx=(8, 0))
