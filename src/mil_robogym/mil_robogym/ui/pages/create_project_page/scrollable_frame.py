import tkinter as tk
from tkinter import ttk


class ScrollableFrame(tk.Frame):
    """
    A vertical scrollable frame for the topics list in the create project frame
    """

    def __init__(self, parent, bg="#DADADA", *args, **kwargs):
        """
        Initialize a canvas-backed frame with a vertical scrollbar.

        The instance exposes `self.content` as the child frame where callers add
        widgets. The canvas automatically tracks content size and keeps content
        width synchronized with the visible viewport.

        :param parent: Parent Tkinter widget that owns this frame.
        :param bg: Background color applied to the container, canvas, and content frame.
        :param args: Additional positional arguments forwarded to `tk.Frame`.
        :param kwargs: Additional keyword arguments forwarded to `tk.Frame`.
        """
        super().__init__(parent, bg=bg, *args, **kwargs)

        self.canvas = tk.Canvas(self, bg=bg, highlightthickness=0, bd=0)
        self.vscroll = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vscroll.set)

        self.vscroll.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.content = tk.Frame(self.canvas, bg=bg)
        self._win = self.canvas.create_window((0, 0), window=self.content, anchor="nw")

        self.content.bind("<Configure>", self._on_content_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

    def _on_content_configure(self, _event):
        """
        Update the canvas scroll region when content size changes.

        :param _event: Tkinter configure event for the content frame.
        """
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        """
        Keep the embedded content frame width equal to canvas viewport width.

        :param event: Tkinter configure event containing updated canvas width.
        """
        self.canvas.itemconfigure(self._win, width=event.width)
