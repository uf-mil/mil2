from __future__ import annotations

import tkinter as tk
from typing import Callable, Literal

from mil_robogym.data_collection.topic_warnings import warn_for_unhelpful_topics
from mil_robogym.data_collection.types import TopicWarning
from mil_robogym.ui.components.scrollable_frame import ScrollableFrame

TopicListType = Literal["input", "output"]


_CATEGORY_LABELS: dict[str, str] = {
    "system_topic": "System topic",
    "sim_internal": "Simulation-only topic",
    "debug_or_stats": "Debug/statistics topic",
    "raw_sensor_stream": "Raw sensor stream",
}


class _HoverToolTip:
    """Show warning text while the pointer is over a widget."""

    def __init__(self, widget: tk.Widget, text: str) -> None:
        self._widget = widget
        self._text = text
        self._tip: tk.Toplevel | None = None

        self._widget.bind("<Enter>", self._show)
        self._widget.bind("<Leave>", self._hide)
        self._widget.bind("<Destroy>", self._hide)

    def _show(self, _: tk.Event) -> None:
        self._hide()

        x = self._widget.winfo_rootx() + 18
        y = self._widget.winfo_rooty() + 20

        tip = tk.Toplevel(self._widget)
        tip.wm_overrideredirect(True)
        tip.attributes("-topmost", True)
        tip.geometry(f"+{x}+{y}")

        label = tk.Label(
            tip,
            text=self._text,
            bg="#2B2B2B",
            fg="white",
            font=("Arial", 10),
            justify="left",
            wraplength=420,
            padx=8,
            pady=4,
        )
        label.pack()
        self._tip = tip

    def _hide(self, _: tk.Event | None = None) -> None:
        if self._tip is not None:
            self._tip.destroy()
            self._tip = None


def _format_topic_warning_message(warning: TopicWarning) -> str:
    category = warning["category"]
    label = _CATEGORY_LABELS.get(category, category.replace("_", " ").title())
    matches = warning.get("matched", [])
    if matches:
        return f"{label}"
    return f"{label}, this topic is likely unhelpful for training."


class TopicsSection:
    """Render and manage the input/output topic selector lists."""

    def __init__(
        self,
        parent: tk.Widget,
        topics: list[str],
        on_selection_change: Callable[[], None],
    ) -> None:
        self._on_selection_change = on_selection_change

        self.input_topics_label = tk.Label(
            parent,
            text="Input Topics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 16, "bold"),
            anchor="w",
        )
        self.input_topics_label.grid(
            row=6,
            column=0,
            columnspan=2,
            sticky="w",
            padx=14,
            pady=(6, 2),
        )

        self.output_topics_label = tk.Label(
            parent,
            text="Output Topics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 16, "bold"),
            anchor="w",
        )
        self.output_topics_label.grid(
            row=6,
            column=3,
            columnspan=3,
            sticky="w",
            padx=(8, 14),
            pady=(6, 2),
        )

        self.outer = tk.Frame(parent, bg="#DADADA")
        self.outer.grid(
            row=7,
            column=0,
            columnspan=6,
            sticky="nsew",
            padx=14,
            pady=(0, 8),
        )
        self.outer.grid_columnconfigure(0, weight=1)
        self.outer.grid_columnconfigure(1, weight=1)
        self.outer.grid_rowconfigure(0, weight=1)

        self.input_topic_frame = ScrollableFrame(self.outer, bg="#DADADA")
        self.input_topic_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.output_topic_frame = ScrollableFrame(self.outer, bg="#DADADA")
        self.output_topic_frame.grid(row=0, column=1, sticky="nsew")

        safe_topics = topics if topics else ["No topics found"]
        warnings = warn_for_unhelpful_topics(safe_topics)
        self._warned_topics = {warning["topic"] for warning in warnings}
        self._topic_warning_messages = {
            warning["topic"]: _format_topic_warning_message(warning)
            for warning in warnings
        }
        self._warning_tooltips: list[_HoverToolTip] = []

        self.input_topic_order = safe_topics.copy()
        self.output_topic_order = safe_topics.copy()

        self.input_topic_vars: dict[str, tk.BooleanVar] = {}
        self.output_topic_vars: dict[str, tk.BooleanVar] = {}
        self.input_topic_buttons: dict[str, tk.Checkbutton] = {}
        self.output_topic_buttons: dict[str, tk.Checkbutton] = {}
        self.input_topic_rows: dict[str, tk.Frame] = {}
        self.output_topic_rows: dict[str, tk.Frame] = {}

        self._build_topic_checkboxes("input")
        self._build_topic_checkboxes("output")

    def _build_topic_checkboxes(self, list_type: TopicListType) -> None:
        if list_type == "input":
            frame = self.input_topic_frame.content
            topic_order = self.input_topic_order
            topic_vars = self.input_topic_vars
            topic_buttons = self.input_topic_buttons
            topic_rows = self.input_topic_rows
        else:
            frame = self.output_topic_frame.content
            topic_order = self.output_topic_order
            topic_vars = self.output_topic_vars
            topic_buttons = self.output_topic_buttons
            topic_rows = self.output_topic_rows

        for topic in topic_order:
            topic_var = topic_vars.get(topic, tk.BooleanVar(value=False))
            topic_vars[topic] = topic_var

            row = tk.Frame(frame, bg="#DADADA")
            button = tk.Checkbutton(
                row,
                text=topic,
                variable=topic_var,
                command=lambda t=topic, lt=list_type: self._on_topic_clicked(t, lt),
                bg="#DADADA",
                fg="black",
                activebackground="#DADADA",
                font=("Arial", 14),
                anchor="w",
                highlightthickness=0,
            )
            button.pack(side="left", anchor="w")

            if topic in self._warned_topics:
                warning_icon = tk.Label(
                    row,
                    text=" \u26a0",
                    bg="#DADADA",
                    fg="red",
                    font=("Arial", 14, "bold"),
                    anchor="w",
                    cursor="hand2",
                )
                warning_icon.pack(side="left", anchor="w")
                warning_message = self._topic_warning_messages.get(
                    topic,
                    "This topic may be unhelpful for training.",
                )
                self._warning_tooltips.append(
                    _HoverToolTip(warning_icon, warning_message),
                )

            row.pack(anchor="w")
            topic_rows[topic] = row
            topic_buttons[topic] = button

    def _on_topic_clicked(self, topic: str, list_type: TopicListType) -> None:
        if list_type == "input":
            topic_order = self.input_topic_order
            topic_rows = self.input_topic_rows
        else:
            topic_order = self.output_topic_order
            topic_rows = self.output_topic_rows

        if topic not in topic_order:
            return

        topic_order.remove(topic)
        topic_order.insert(0, topic)

        for ordered_topic in topic_order:
            row = topic_rows[ordered_topic]
            row.pack_forget()
            row.pack(anchor="w")

        self._on_selection_change()

    def selected_topics(self, list_type: TopicListType) -> list[str]:
        if list_type == "input":
            topic_order = self.input_topic_order
            topic_vars = self.input_topic_vars
        else:
            topic_order = self.output_topic_order
            topic_vars = self.output_topic_vars

        return [topic for topic in topic_order if topic_vars[topic].get()]

    def get_selected_input_topics(self) -> list[str]:
        return self.selected_topics("input")

    def get_selected_output_topics(self) -> list[str]:
        return self.selected_topics("output")

    def has_valid_topic_selection(self) -> bool:
        return bool(self.get_selected_input_topics()) and bool(
            self.get_selected_output_topics(),
        )
