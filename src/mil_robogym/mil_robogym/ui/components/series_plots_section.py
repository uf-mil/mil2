from __future__ import annotations

import csv
import tkinter as tk
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator


class SeriesPlotsSection:
    """Reusable multi-plot panel for live or CSV-backed scalar series."""

    def __init__(
        self,
        parent: tk.Widget,
        *,
        title: str,
        preferred_metric_order: Sequence[str] = (),
        section_bg: str = "#DADADA",
        content_bg: str = "#F2F2F2",
        empty_state_message: str = "No data available.",
        pending_data_message: str = "Waiting for data...",
        missing_file_message: str = "No data file found.",
        empty_file_message: str = "Data file is empty.",
        no_selection_message: str = "Use + to add metrics.",
        label_formatter: Callable[[str], str] | None = None,
    ) -> None:
        self._metric_data: dict[str, list[float]] = {}
        self._selected_metrics: list[str] = []
        self._preferred_metric_order = list(dict.fromkeys(preferred_metric_order))
        self._x_metric_name: str | None = None
        self._metric_controls_signature: (
            tuple[tuple[str, ...], tuple[str, ...]] | None
        ) = None
        self._selected_sample_index: int | None = None
        self._placeholder_text = empty_state_message
        self._empty_state_message = empty_state_message
        self._pending_data_message = pending_data_message
        self._missing_file_message = missing_file_message
        self._empty_file_message = empty_file_message
        self._no_selection_message = no_selection_message
        self._label_formatter = label_formatter or self._default_label_formatter
        self._resize_after_id: str | None = None

        self.container = tk.Frame(parent, bg=section_bg)
        self.container.grid_columnconfigure(0, weight=1)
        self.container.grid_rowconfigure(2, weight=1)

        header = tk.Frame(self.container, bg=section_bg)
        header.grid(row=0, column=0, sticky="ew")
        header.grid_columnconfigure(0, weight=1)

        self.title_label = tk.Label(
            header,
            text=title,
            bg=section_bg,
            fg="black",
            font=("Arial", 18, "bold"),
            anchor="w",
        )
        self.title_label.grid(row=0, column=0, sticky="w", pady=(0, 4))

        self.add_metric_menu = tk.Menu(header, tearoff=False)
        self.add_metric_menu.bind("<FocusOut>", self._dismiss_add_metric_menu)
        self.add_metric_menu.bind("<Escape>", self._dismiss_add_metric_menu)
        self.add_metric_button = tk.Button(
            header,
            text="+",
            command=self._show_add_metric_menu,
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

        self.selected_metrics_frame = tk.Frame(self.container, bg=section_bg)
        self.selected_metrics_frame.grid(row=1, column=0, sticky="ew", pady=(0, 4))
        self.selected_metrics_frame.grid_columnconfigure(0, weight=1)

        self.content_frame = tk.Frame(
            self.container,
            bg=content_bg,
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
            bg=content_bg,
            fg="#444444",
            font=("Arial", 10),
            pady=20,
        )
        self.placeholder_label.grid(row=0, column=0, sticky="nsew")

        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.content_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=0, column=0, sticky="nsew")

        self.clear_metrics()

    def clear_metrics(self, message: str | None = None) -> None:
        """Reset the panel to a placeholder state."""
        self._metric_data = {}
        self._selected_metrics = []
        self._x_metric_name = None
        self._selected_sample_index = None
        self._placeholder_text = message or self._empty_state_message
        self._render_metric_controls(force=True)
        self._render_plots()

    def get_selected_metrics(self) -> list[str]:
        """Return the current metric watchlist."""
        return list(self._selected_metrics)

    def set_selected_metrics(self, metric_names: Sequence[str]) -> None:
        """Seed or restore a preferred watchlist."""
        self._selected_metrics = list(dict.fromkeys(metric_names))
        if self._metric_data:
            self._render_metric_controls()
            self._render_plots()

    def set_metric_order(self, metric_names: Sequence[str]) -> None:
        """Update the preferred series ordering used in the + menu and defaults."""
        self._preferred_metric_order = list(dict.fromkeys(metric_names))
        if self._metric_data:
            self._render_metric_controls()
            self._render_plots()

    def set_selected_sample_index(self, index: int | None) -> None:
        """Highlight one sample position across all rendered plots."""
        normalized_index = index if index is None or index >= 0 else None
        if normalized_index == self._selected_sample_index:
            return
        self._selected_sample_index = normalized_index
        self._render_plots()

    def set_metrics_data(
        self,
        metrics: Mapping[str, Sequence[float]] | None,
    ) -> None:
        """Render series data from an in-memory mapping."""
        if not metrics or not any(len(series) > 0 for series in metrics.values()):
            self.clear_metrics(self._pending_data_message)
            return

        self._metric_data = {
            name: [float(value) for value in series] for name, series in metrics.items()
        }
        self._x_metric_name = "episode" if "episode" in self._metric_data else None
        self._sync_selected_metrics()
        self._placeholder_text = self._no_selection_message
        self._render_metric_controls()
        self._render_plots()

    def append_metric_point(self, metric_values: Mapping[str, float | int]) -> None:
        """Append one live data point to the in-memory series and redraw."""
        if not metric_values:
            return

        for metric_name, raw_value in metric_values.items():
            self._metric_data.setdefault(metric_name, []).append(float(raw_value))

        self._sync_selected_metrics()
        self._placeholder_text = self._no_selection_message
        self._render_metric_controls()
        self._render_plots()

    def load_metrics_csv(self, csv_path: Path | None) -> None:
        """Read a numeric CSV file and render its columns as live plots."""
        if csv_path is None or not csv_path.is_file():
            self.clear_metrics(self._missing_file_message)
            return

        with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            if not reader.fieldnames:
                self.clear_metrics(self._empty_file_message)
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

    def _sync_selected_metrics(self) -> None:
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

    def _available_metric_names(self) -> list[str]:
        metric_names = [
            metric_name
            for metric_name in self._metric_data
            if metric_name not in {"index", self._x_metric_name}
        ]
        preferred_names = [
            metric_name
            for metric_name in self._preferred_metric_order
            if metric_name in metric_names
        ]
        extra_names = sorted(
            metric_name
            for metric_name in metric_names
            if metric_name not in preferred_names
        )
        return [*preferred_names, *extra_names]

    def _default_selected_metrics(self, available_metrics: Sequence[str]) -> list[str]:
        return list(available_metrics)

    def _metric_controls_state_signature(
        self,
    ) -> tuple[tuple[str, ...], tuple[str, ...]]:
        available_metrics = self._available_metric_names()
        remaining_metrics = [
            metric_name
            for metric_name in available_metrics
            if metric_name not in self._selected_metrics
        ]
        return tuple(self._selected_metrics), tuple(remaining_metrics)

    def _render_metric_controls(self, *, force: bool = False) -> None:
        signature = self._metric_controls_state_signature()
        if not force and signature == self._metric_controls_signature:
            return

        self._metric_controls_signature = signature
        _, remaining_metrics = signature

        for child in self.selected_metrics_frame.winfo_children():
            child.destroy()

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

    def _dismiss_add_metric_menu(self, _event: tk.Event | None = None) -> None:
        self.add_metric_menu.unpost()

    def _show_add_metric_menu(self) -> None:
        if str(self.add_metric_button.cget("state")) == "disabled":
            return

        try:
            self.add_metric_menu.tk_popup(
                self.add_metric_button.winfo_rootx(),
                self.add_metric_button.winfo_rooty()
                + self.add_metric_button.winfo_height(),
            )
        finally:
            self.add_metric_menu.grab_release()

    def _add_metric(self, metric_name: str) -> None:
        if metric_name in self._selected_metrics:
            return
        self._selected_metrics.append(metric_name)
        self._render_metric_controls(force=True)
        self._render_plots()

    def _remove_metric(self, metric_name: str) -> None:
        self._selected_metrics = [
            selected_metric
            for selected_metric in self._selected_metrics
            if selected_metric != metric_name
        ]
        self._render_metric_controls(force=True)
        self._render_plots()

    def _render_plots(self) -> None:
        if not self._metric_data:
            self._show_placeholder(self._placeholder_text)
            return

        if not self._selected_metrics:
            self._show_placeholder(self._no_selection_message)
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
            axis.set_facecolor("#FFFFFF")
            axis.plot(
                x_values[: len(y_values)],
                y_values,
                linewidth=2.0,
                color="#2B5D83",
            )
            axis.grid(True, axis="y", alpha=0.24, color="#8C98A5", linewidth=0.7)
            axis.grid(True, axis="x", alpha=0.18, color="#A6B0BA", linewidth=0.6)
            axis.margins(x=0.025, y=0.18)
            axis.yaxis.set_major_locator(MaxNLocator(nbins=4, min_n_ticks=3))
            axis.tick_params(axis="y", labelsize=8, pad=2, colors="#4D5560")
            axis.tick_params(
                axis="x",
                labelsize=8,
                pad=-11,
                length=0,
                colors="#4D5560",
            )
            axis.spines["top"].set_visible(False)
            axis.spines["right"].set_visible(False)
            axis.spines["left"].set_color("#B7BEC6")
            axis.spines["bottom"].set_color("#B7BEC6")
            axis.spines["left"].set_linewidth(0.8)
            axis.spines["bottom"].set_linewidth(0.8)
            axis.text(
                0.018,
                0.94,
                self._label_formatter(metric_name),
                transform=axis.transAxes,
                ha="left",
                va="top",
                fontsize=8,
                fontweight="bold",
                color="#1F2A33",
                bbox={
                    "boxstyle": "round,pad=0.22",
                    "facecolor": "#F6F8FA",
                    "edgecolor": "#D7DDE3",
                    "linewidth": 0.6,
                },
            )
            selected_x_value = self._selected_x_value(x_values)
            if selected_x_value is not None:
                axis.axvline(
                    x=selected_x_value,
                    color="#C75C1A",
                    linestyle="--",
                    linewidth=1.2,
                    alpha=0.9,
                    zorder=5,
                )
            axis.label_outer()

        axes[-1].set_xlabel("")
        axes[-1].text(
            0.985,
            0.055,
            self._label_formatter(self._x_metric_name or "index"),
            transform=axes[-1].transAxes,
            ha="right",
            va="bottom",
            fontsize=8,
            color="#4D5560",
            bbox={
                "boxstyle": "round,pad=0.18",
                "facecolor": "#FFFFFF",
                "edgecolor": "#D7DDE3",
                "linewidth": 0.6,
                "alpha": 0.96,
            },
        )
        self.figure.subplots_adjust(
            left=0.074,
            right=0.985,
            top=0.988,
            bottom=0.06,
            hspace=0.12,
        )
        self.canvas.draw_idle()

    def _x_values(self) -> list[float]:
        if self._x_metric_name and self._x_metric_name in self._metric_data:
            return self._metric_data[self._x_metric_name]
        if not self._metric_data:
            return []
        first_series = next(iter(self._metric_data.values()), [])
        return [float(index) for index in range(len(first_series))]

    def _selected_x_value(self, x_values: Sequence[float]) -> float | None:
        if self._selected_sample_index is None:
            return None
        if not (0 <= self._selected_sample_index < len(x_values)):
            return None
        return float(x_values[self._selected_sample_index])

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

    @staticmethod
    def _default_label_formatter(metric_name: str) -> str:
        return metric_name.replace("_", " ").upper()
