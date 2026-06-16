import tkinter as tk
from tkinter import ttk

from .mouse_wheel import get_mouse_wheel_router, scroll_canvas


class ScrollableFrame(tk.Frame):
    """
    A vertical scrollable frame for the topics list in the create project frame
    """

    def __init__(
        self,
        parent,
        bg="#DADADA",
        *,
        fill_height: bool = False,
        **kwargs,
    ):
        """
        Initialize a canvas-backed frame with a vertical scrollbar.

        The instance exposes `self.content` as the child frame where callers add
        widgets. The canvas automatically tracks content size and keeps content
        width synchronized with the visible viewport.

        :param parent: Parent Tkinter widget that owns this frame.
        :param bg: Background color applied to the container, canvas, and content frame.
        :param fill_height: When `True`, keep the embedded content at least as
            tall as the visible viewport so grid/pack layouts can still expand
            vertically when content does not overflow.
        :param kwargs: Additional keyword arguments forwarded to `tk.Frame`.
        """
        super().__init__(parent, bg=bg, **kwargs)
        self._fill_height = fill_height

        self.canvas = tk.Canvas(self, bg=bg, highlightthickness=0, bd=0)
        self.vscroll = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vscroll.set)

        self.vscroll.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.content = tk.Frame(self.canvas, bg=bg)
        self._win = self.canvas.create_window((0, 0), window=self.content, anchor="nw")

        self.content.bind("<Configure>", self._on_content_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self._mouse_wheel_router = get_mouse_wheel_router(self)
        self._mouse_wheel_binding = self._mouse_wheel_router.register(
            self,
            self._handle_mouse_wheel,
        )

    def reset_scroll(self) -> None:
        """Scroll the viewport back to the top of the content."""
        self.canvas.yview_moveto(0.0)

    def _on_content_configure(self, _event):
        """
        Update the canvas scroll region when content size changes.

        :param _event: Tkinter configure event for the content frame.
        """
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        self._sync_window_size()

    def _on_canvas_configure(self, event):
        """
        Keep the embedded content frame width equal to canvas viewport width.

        :param event: Tkinter configure event containing updated canvas width.
        """
        self._sync_window_size(width=event.width, height=event.height)

    def _sync_window_size(
        self,
        *,
        width: int | None = None,
        height: int | None = None,
    ) -> None:
        canvas_width = width if width is not None else self.canvas.winfo_width()
        if canvas_width > 1:
            self.canvas.itemconfigure(self._win, width=canvas_width)

        if not self._fill_height:
            return

        canvas_height = height if height is not None else self.canvas.winfo_height()
        if canvas_height <= 1:
            return

        content_height = self.content.winfo_reqheight()
        self.canvas.itemconfigure(
            self._win,
            height=max(content_height, canvas_height),
        )

    def _handle_mouse_wheel(self, units: int) -> bool:
        return scroll_canvas(self.canvas, units)
