import tkinter as tk
from contextlib import suppress
from typing import Optional


class ToolTip:
    """
    A small hover tooltip component that displays contextual information
    when the mouse enters a widget and disappears when it leaves.
    """

    def __init__(
        self,
        widget: tk.Widget,
        text: str,
        *,
        group: "ToolTipGroup | None" = None,
        forced_text: str | None = None,
    ) -> None:
        """
        Initialize the tooltip.

        Args:
            widget: The tkinter widget this tooltip is attached to.
            text: The text displayed inside the tooltip.
        """
        self.widget: tk.Widget = widget
        self.text: str = text
        self.forced_text: str = forced_text or text
        self.tip: Optional[tk.Toplevel] = None
        self.label: Optional[tk.Label] = None
        self._hover_active = False
        self._forced_visible = False

        widget.bind("<Enter>", self.show, add="+")
        widget.bind("<Leave>", self.hide, add="+")
        widget.bind("<Destroy>", self._on_destroy, add="+")

        if group is not None:
            group.register(self)

    def show(self, _event: tk.Event | None = None) -> None:
        """
        Display the tooltip near the widget.
        """
        self._hover_active = True
        self._sync_visibility()

    def hide(self, _event: tk.Event | None = None) -> None:
        """
        Destroy the tooltip window if it exists.
        """
        self._hover_active = False
        self._sync_visibility()

    def force_show(self) -> None:
        """Display the tooltip even when the mouse is not hovering."""
        self._forced_visible = True
        self._sync_visibility()

    def force_hide(self) -> None:
        """Hide the forced tooltip state while preserving hover behavior."""
        self._forced_visible = False
        self._sync_visibility()

    def _sync_visibility(self) -> None:
        if self._hover_active and self._is_visible():
            self._show_tip(mode="hover")
            return
        if self._forced_visible and self._is_visible():
            self._show_tip(mode="forced")
            return
        self._hide_tip()

    def _is_visible(self) -> bool:
        with suppress(tk.TclError):
            return bool(self.widget.winfo_exists() and self.widget.winfo_viewable())
        return False

    def _ensure_tip(self) -> None:
        if self.tip is not None and self.label is not None:
            return

        self.tip = tk.Toplevel(self.widget)
        self.tip.withdraw()
        self.tip.wm_overrideredirect(True)

        self.label = tk.Label(self.tip)
        self.label.pack()

    def _show_tip(self, *, mode: str) -> None:
        self._ensure_tip()
        self.widget.update_idletasks()

        if mode == "forced":
            text = self.forced_text
            self.label.configure(
                text=text,
                bg="#F8FAFC",
                fg="#111827",
                padx=5,
                pady=2,
                font=("Arial", 8, "bold"),
                relief="solid",
                bd=1,
                highlightthickness=0,
            )
        else:
            text = self.text
            self.label.configure(
                text=text,
                bg="#111827",
                fg="#F9FAFB",
                padx=7,
                pady=4,
                font=("Arial", 9),
                relief="solid",
                bd=1,
                highlightthickness=0,
            )

        self.tip.withdraw()
        self.tip.update_idletasks()
        width = self.label.winfo_reqwidth()
        height = self.label.winfo_reqheight()

        x = self.widget.winfo_rootx() + max((self.widget.winfo_width() - width) // 2, 0)
        if mode == "forced":
            y = self.widget.winfo_rooty() - height - 6
            if y < 0:
                y = self.widget.winfo_rooty() + self.widget.winfo_height() + 6
        else:
            y = self.widget.winfo_rooty() + self.widget.winfo_height() + 8

        self.tip.geometry(f"+{x}+{y}")
        self.tip.deiconify()
        self.tip.lift()

    def _hide_tip(self) -> None:
        if self.tip is not None:
            self.tip.withdraw()

    def _destroy_tip(self) -> None:
        if self.tip is not None:
            self.tip.destroy()
            self.tip = None
            self.label = None

    def _on_destroy(self, _event: tk.Event | None = None) -> None:
        self._hover_active = False
        self._forced_visible = False
        self._destroy_tip()


class ToolTipGroup:
    """Manage a related set of tooltips so they can be shown together."""

    def __init__(self) -> None:
        self._tooltips: list[ToolTip] = []

    def register(self, tooltip: ToolTip) -> None:
        self._tooltips.append(tooltip)

    def add(
        self,
        widget: tk.Widget,
        text: str,
        *,
        forced_text: str | None = None,
    ) -> ToolTip:
        return ToolTip(widget, text, group=self, forced_text=forced_text)

    def show_all(self) -> None:
        live_tooltips: list[ToolTip] = []
        for tooltip in self._tooltips:
            with suppress(tk.TclError):
                if tooltip.widget.winfo_exists():
                    live_tooltips.append(tooltip)
        self._tooltips = live_tooltips
        for tooltip in self._tooltips:
            tooltip.force_show()

    def hide_all(self) -> None:
        for tooltip in self._tooltips:
            tooltip.force_hide()

    def clear(self) -> None:
        self.hide_all()
        self._tooltips.clear()
