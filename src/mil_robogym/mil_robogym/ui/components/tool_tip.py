import tkinter as tk
from typing import Optional


class ToolTip:
    """
    A small hover tooltip component that displays contextual information
    when the mouse enters a widget and disappears when it leaves.
    """

    def __init__(self, widget: tk.Widget, text: str) -> None:
        """
        Initialize the tooltip.

        Args:
            widget: The tkinter widget this tooltip is attached to.
            text: The text displayed inside the tooltip.
        """
        self.widget: tk.Widget = widget
        self.text: str = text
        self.tip: Optional[tk.Toplevel] = None

        widget.bind("<Enter>", self.show)
        widget.bind("<Leave>", self.hide)

    def show(self, _: tk.Event) -> None:
        """
        Display the tooltip near the widget.
        """
        x, y, _, _ = self.widget.bbox("insert") or (0, 0, 0, 0)
        x += self.widget.winfo_rootx() + 30
        y += self.widget.winfo_rooty() + 30

        self.tip = tk.Toplevel(self.widget)
        self.tip.wm_overrideredirect(True)
        self.tip.geometry(f"+{x}+{y}")

        label = tk.Label(
            self.tip,
            text=self.text,
            bg="black",
            fg="white",
            padx=6,
            pady=3,
        )
        label.pack()

    def hide(self, _: tk.Event) -> None:
        """
        Destroy the tooltip window if it exists.
        """
        if self.tip is not None:
            self.tip.destroy()
            self.tip = None
