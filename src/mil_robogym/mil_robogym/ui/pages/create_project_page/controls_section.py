from __future__ import annotations

import tkinter as tk
from typing import Callable


class ControlsSection:
    """Build and manage the bottom action controls for project creation. This keeps behavior scoped to the current component.

    Args:
        parent: Parent widget that hosts the control buttons.
        on_cancel: Callback run when the Cancel button is pressed.
        on_create_project: Callback run when the Create Project button is pressed.
    Returns:
        None.
    """

    def __init__(
        self,
        parent: tk.Widget,
        on_cancel: Callable[[], None],
        on_create_project: Callable[[], None],
    ) -> None:
        """Create controls widgets and store action callbacks. This keeps behavior scoped to the current component.

        Args:
            parent: Parent widget where this section is rendered.
            on_cancel: Callback used for cancel navigation behavior.
            on_create_project: Callback used for create-project submission behavior.
        Returns:
            None.
        """
        self._on_cancel = on_cancel
        self._on_create_project = on_create_project

        self._create_project_error_tip: tk.Toplevel | None = None
        self._create_project_error_tip_after_id: str | None = None

        self.cancel_button = tk.Button(
            parent,
            text="Cancel",
            command=self._on_cancel_click,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 16),
            padx=10,
            pady=6,
        )
        self.cancel_button.grid(
            row=11,
            column=0,
            columnspan=3,
            sticky="nsew",
            padx=(14, 8),
            pady=(8, 14),
        )

        self.create_project_button = tk.Button(
            parent,
            text="Create Project",
            command=self._on_create_project_click,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 16),
            padx=10,
            pady=6,
        )
        self.create_project_button.grid(
            row=11,
            column=3,
            columnspan=3,
            sticky="nsew",
            padx=(8, 14),
            pady=(8, 14),
        )

    def _on_cancel_click(self) -> None:
        """Handle Cancel button presses and dispatch to the provided callback. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        self._on_cancel()

    def _on_create_project_click(self) -> None:
        """Handle Create Project button presses and dispatch to the provided callback. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        self._on_create_project()

    def set_create_project_enabled(self, enabled: bool) -> None:
        """Apply enabled or disabled styling to the Create Project button. This keeps behavior scoped to the current component.

        Args:
            enabled: Whether the create button should be interactive.
        Returns:
            None.
        """
        if enabled:
            self.create_project_button.configure(
                state=tk.NORMAL,
                text="Create Project",
                fg="black",
                bg="#ECECEC",
                activebackground="#DFDFDF",
            )
            self.hide_error_tooltip()
            return

        self.create_project_button.configure(
            state=tk.DISABLED,
            text="Create Project",
            fg="#888888",
            bg="#E0E0E0",
            activebackground="#E0E0E0",
            disabledforeground="#888888",
        )
        self.hide_error_tooltip()

    def update_create_project_button_state(self, enabled: bool) -> None:
        """Update button enabled state from current form validity. This keeps behavior scoped to the current component.

        Args:
            enabled: Whether the form is currently valid.
        Returns:
            None.
        """
        self.set_create_project_enabled(enabled)

    def show_error_tooltip(self, message: str) -> None:
        """Show a transient tooltip above the Create Project button. This keeps behavior scoped to the current component.

        Args:
            message: User-facing error message rendered in the tooltip.
        Returns:
            None.
        """
        self.hide_error_tooltip()

        tip = tk.Toplevel(self.create_project_button)
        tip.wm_overrideredirect(True)
        tip.attributes("-topmost", True)

        x = self.create_project_button.winfo_rootx()
        y = self.create_project_button.winfo_rooty() - 30
        tip.geometry(f"+{x}+{y}")

        message_label = tk.Label(
            tip,
            text=message,
            bg="#2B2B2B",
            fg="white",
            font=("Arial", 10),
            padx=8,
            pady=4,
        )
        message_label.pack()

        self._create_project_error_tip = tip
        self._create_project_error_tip_after_id = self.create_project_button.after(
            4000,
            self.hide_error_tooltip,
        )

    def hide_error_tooltip(self) -> None:
        """Hide any active Create Project tooltip and clear pending timers. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        if self._create_project_error_tip_after_id is not None:
            self.create_project_button.after_cancel(
                self._create_project_error_tip_after_id,
            )
            self._create_project_error_tip_after_id = None

        if self._create_project_error_tip is not None:
            self._create_project_error_tip.destroy()
            self._create_project_error_tip = None
