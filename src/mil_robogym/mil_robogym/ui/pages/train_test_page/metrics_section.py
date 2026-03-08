from __future__ import annotations

import tkinter as tk
from pathlib import Path


class MetricsSection:
    """Scrollable metrics image panel."""

    def __init__(self, parent: tk.Widget) -> None:
        self._image_refs: list[tk.PhotoImage] = []

        self.container = tk.Frame(parent, bg="#DADADA")
        self.container.grid(
            row=1,
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
            text="Metrics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 18, "bold"),
            anchor="w",
        )
        self.title_label.grid(row=0, column=0, sticky="w", pady=(0, 4))

        self.content_frame = tk.Frame(
            self.container,
            bg="#F2F2F2",
            relief="solid",
            bd=1,
            padx=8,
            pady=8,
        )
        self.content_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")
        self.content_frame.grid_columnconfigure(0, weight=1)
        self.content_frame.grid_rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(
            self.content_frame,
            bg="#F2F2F2",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")

        self.scrollbar = tk.Scrollbar(
            self.content_frame,
            orient="vertical",
            command=self.canvas.yview,
        )
        self.scrollbar.grid(row=0, column=1, sticky="ns")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.scrollable_frame = tk.Frame(self.canvas, bg="#F2F2F2")
        self.canvas_window = self.canvas.create_window(
            (0, 0),
            window=self.scrollable_frame,
            anchor="nw",
        )

        self.scrollable_frame.bind("<Configure>", self._on_frame_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

        self.placeholder_label = tk.Label(
            self.scrollable_frame,
            text="Table/Graph of collected input and output data",
            bg="#F2F2F2",
            fg="#444444",
            font=("Arial", 10),
            pady=20,
        )
        self.placeholder_label.pack(fill="x")

    def load_metrics(self, metrics_dir: Path | None) -> None:
        """Load all image files from metrics directory into scrollable panel."""
        for child in self.scrollable_frame.winfo_children():
            child.destroy()
        self._image_refs.clear()

        if metrics_dir is None or not metrics_dir.is_dir():
            self._render_text_placeholder("No metrics directory found.")
            return

        image_paths = sorted(
            [
                path
                for path in metrics_dir.iterdir()
                if path.is_file()
                and path.suffix.lower() in {".png", ".gif", ".ppm", ".pgm"}
            ],
        )
        if not image_paths:
            self._render_text_placeholder("No metric images found.")
            return

        for image_path in image_paths:
            try:
                image = tk.PhotoImage(file=str(image_path))
            except tk.TclError:
                continue

            self._image_refs.append(image)
            card = tk.Frame(self.scrollable_frame, bg="#FFFFFF", relief="solid", bd=1)
            card.pack(fill="x", padx=10, pady=6)

            title = tk.Label(
                card,
                text=image_path.name,
                bg="#FFFFFF",
                fg="black",
                font=("Arial", 10, "bold"),
                anchor="w",
                padx=8,
                pady=6,
            )
            title.pack(fill="x")

            image_label = tk.Label(card, image=image, bg="#FFFFFF")
            image_label.pack(padx=8, pady=(0, 8))

        if not self._image_refs:
            self._render_text_placeholder(
                "No supported metric images could be displayed.",
            )

    def _render_text_placeholder(self, text: str) -> None:
        """Render placeholder text when no images are available."""
        label = tk.Label(
            self.scrollable_frame,
            text=text,
            bg="#F2F2F2",
            fg="#444444",
            font=("Arial", 10),
            pady=20,
        )
        label.pack(fill="x")

    def _on_frame_configure(self, _event: tk.Event | None = None) -> None:
        """Refresh canvas scroll region when content changes."""
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event: tk.Event) -> None:
        """Keep scrollable frame width equal to canvas width."""
        self.canvas.itemconfigure(self.canvas_window, width=event.width)
