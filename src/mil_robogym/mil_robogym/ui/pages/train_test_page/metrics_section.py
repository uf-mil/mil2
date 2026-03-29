from __future__ import annotations

import csv
import tkinter as tk
from collections.abc import Mapping, Sequence
from pathlib import Path

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

DEFAULT_METRIC_ORDER = (
    "reward_mean",
    "reward_std",
    "disc_loss",
    "disc_kl",
    "disc_beta",
)


class MetricsSection:
    """Resizable metrics panel that renders live or saved series directly."""

    def __init__(self, parent: tk.Widget) -> None:
        self._metric_data: dict[str, list[float]] = {}
        self._selected_metrics: list[str] = []
        self._x_metric_name: str | None = None
        self._placeholder_text = "No metrics selected."
        self._resize_after_id: str | None = None

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
        self.container.grid_rowconfigure(2, weight=1)

        header = tk.Frame(self.container, bg="#DADADA")
        header.grid(row=0, column=0, sticky="ew")
        header.grid_columnconfigure(0, weight=1)

        self.title_label = tk.Label(
            header,
            text="Metrics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 18, "bold"),
            anchor="w",
        )
        self.title_label.grid(row=0, column=0, sticky="w", pady=(0, 4))

        self.add_metric_menu = tk.Menu(header, tearoff=False)
        self.add_metric_button = tk.Menubutton(
            header,
            text="+",
            menu=self.add_metric_menu,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 12, "bold"),
            padx=8,
            pady=0,
            cursor="hand2",
        )
        self.add_metric_button.grid(row=0, column=1, sticky="e", pady=(0, 4))
        self.add_metric_button["menu"] = self.add_metric_menu

        self.selected_metrics_frame = tk.Frame(self.container, bg="#DADADA")
        self.selected_metrics_frame.grid(row=1, column=0, sticky="ew", pady=(0, 4))
        self.selected_metrics_frame.grid_columnconfigure(0, weight=1)

        self.content_frame = tk.Frame(
            self.container,
            bg="#F2F2F2",
            relief="solid",
            bd=1,
            padx=8,
            pady=8,
        )
        self.content_frame.grid(row=2, column=0, sticky="nsew")
        self.content_frame.grid_columnconfigure(0, weight=1)
        self.content_frame.grid_rowconfigure(0, weight=1)
        self.content_frame.bind("<Configure>", self._on_content_resize)

        self.placeholder_label = tk.Label(
            self.content_frame,
            text=self._placeholder_text,
            bg="#F2F2F2",
            fg="#444444",
            font=("Arial", 10),
            pady=20,
        )
        self.placeholder_label.grid(row=0, column=0, sticky="nsew")

        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.content_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=0, column=0, sticky="nsew")

        self.clear_metrics("No training metrics available.")

    def clear_metrics(self, message: str) -> None:
        """Reset the panel to a placeholder state."""
        self._metric_data = {}
        self._selected_metrics = []
        self._x_metric_name = None
        self._placeholder_text = message
        self._render_metric_controls()
        self._render_plots()

    def set_metrics_data(
        self,
        metrics: Mapping[str, Sequence[float]] | None,
    ) -> None:
        """Render metrics from an in-memory series mapping."""
        if not metrics:
            self.clear_metrics("Waiting for training metrics...")
            return

        normalized: dict[str, list[float]] = {
            name: [float(value) for value in series] for name, series in metrics.items()
        }
        self._metric_data = normalized
        self._x_metric_name = "episode" if "episode" in normalized else None

        available_metrics = self._available_metric_names()
        preserved_selection = [
            metric_name
            for metric_name in self._selected_metrics
            if metric_name in available_metrics
        ]
        self._selected_metrics = (
            preserved_selection
            if preserved_selection
            else self._default_selected_metrics(available_metrics)
        )
        self._placeholder_text = "Use + to add metrics."
        self._render_metric_controls()
        self._render_plots()

    def load_metrics_csv(self, csv_path: Path | None) -> None:
        """Read a saved metrics CSV and render it as live plots."""
        if csv_path is None or not csv_path.is_file():
            self.clear_metrics("No metrics file found.")
            return

        with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            if not reader.fieldnames:
                self.clear_metrics("Metrics file is empty.")
                return

            metrics: dict[str, list[float]] = {
                field_name: []
                for field_name in reader.fieldnames
                if field_name != "index"
            }
            for row in reader:
                for field_name, values in metrics.items():
                    raw_value = row.get(field_name)
                    if raw_value in (None, ""):
                        continue
                    values.append(float(raw_value))

        self.set_metrics_data(metrics)

    def _available_metric_names(self) -> list[str]:
        return [
            metric_name
            for metric_name in self._metric_data
            if metric_name not in {"index", self._x_metric_name}
        ]

    def _default_selected_metrics(self, available_metrics: Sequence[str]) -> list[str]:
        selected = [
            metric_name
            for metric_name in DEFAULT_METRIC_ORDER
            if metric_name in available_metrics
        ][:2]
        if selected:
            return selected
        return list(available_metrics[:2])

    def _render_metric_controls(self) -> None:
        for child in self.selected_metrics_frame.winfo_children():
            child.destroy()

        available_metrics = self._available_metric_names()
        remaining_metrics = [
            metric_name
            for metric_name in available_metrics
            if metric_name not in self._selected_metrics
        ]

        for metric_name in self._selected_metrics:
            chip = tk.Frame(
                self.selected_metrics_frame,
                bg="#ECECEC",
                relief="solid",
                bd=1,
                padx=4,
                pady=2,
            )
            chip.pack(side="left", padx=(0, 6))

            tk.Label(
                chip,
                text=metric_name,
                bg="#ECECEC",
                fg="black",
                font=("Arial", 9),
            ).pack(side="left")

            tk.Button(
                chip,
                text="x",
                command=lambda name=metric_name: self._remove_metric(name),
                bg="#ECECEC",
                activebackground="#DFDFDF",
                fg="black",
                relief="flat",
                bd=0,
                font=("Arial", 9, "bold"),
                padx=4,
                pady=0,
                cursor="hand2",
            ).pack(side="left")

        self.add_metric_menu.delete(0, "end")
        for metric_name in remaining_metrics:
            self.add_metric_menu.add_command(
                label=metric_name,
                command=lambda name=metric_name: self._add_metric(name),
            )
        self.add_metric_button.configure(
            state="normal" if remaining_metrics else "disabled",
        )

    def _add_metric(self, metric_name: str) -> None:
        if metric_name in self._selected_metrics:
            return
        self._selected_metrics.append(metric_name)
        self._render_metric_controls()
        self._render_plots()

    def _remove_metric(self, metric_name: str) -> None:
        self._selected_metrics = [
            selected_metric
            for selected_metric in self._selected_metrics
            if selected_metric != metric_name
        ]
        self._render_metric_controls()
        self._render_plots()

    def _render_plots(self) -> None:
        if not self._metric_data:
            self._show_placeholder(self._placeholder_text)
            return

        if not self._selected_metrics:
            self._show_placeholder("Use + to add metrics.")
            return

        self.placeholder_label.grid_remove()
        self.canvas_widget.grid()

        self.figure.clear()
        self._sync_figure_size()

        subplot_count = len(self._selected_metrics)
        axes = self.figure.subplots(subplot_count, 1, sharex=True)
        if subplot_count == 1:
            axes = [axes]

        x_values = self._x_values()
        for axis, metric_name in zip(axes, self._selected_metrics):
            y_values = self._metric_data.get(metric_name, [])
            axis.plot(
                x_values[: len(y_values)],
                y_values,
                linewidth=2.0,
                color="#2B5D83",
            )
            axis.set_title(metric_name, fontsize=10, loc="left")
            axis.grid(True, alpha=0.3)
            axis.tick_params(labelsize=8)
            axis.set_ylabel(metric_name, fontsize=8)

        axes[-1].set_xlabel(self._x_metric_name or "index", fontsize=9)
        self.figure.tight_layout(pad=1.1)
        self.canvas.draw_idle()

    def _x_values(self) -> list[float]:
        if self._x_metric_name and self._x_metric_name in self._metric_data:
            return self._metric_data[self._x_metric_name]
        if not self._metric_data:
            return []
        first_series = next(iter(self._metric_data.values()), [])
        return [float(index) for index in range(len(first_series))]

    def _show_placeholder(self, text: str) -> None:
        self.placeholder_label.configure(text=text)
        self.canvas_widget.grid_remove()
        self.placeholder_label.grid()
        self.figure.clear()
        self.canvas.draw_idle()

    def _sync_figure_size(self) -> None:
        width = max(self.content_frame.winfo_width() - 16, 1)
        height = max(self.content_frame.winfo_height() - 16, 1)
        self.figure.set_size_inches(
            width / self.figure.dpi,
            height / self.figure.dpi,
            forward=False,
        )

    def _on_content_resize(self, _event: tk.Event | None = None) -> None:
        if self._resize_after_id is not None:
            self.content_frame.after_cancel(self._resize_after_id)
        self._resize_after_id = self.content_frame.after(50, self._resize_render)

    def _resize_render(self) -> None:
        self._resize_after_id = None
        self._render_plots()
