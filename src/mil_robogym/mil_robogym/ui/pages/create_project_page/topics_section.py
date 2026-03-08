from __future__ import annotations

import tkinter as tk
from typing import Callable, Literal

from mil_robogym.ui.components.scrollable_frame import ScrollableFrame

TopicListType = Literal["input", "output"]


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

        self.input_topic_order = safe_topics.copy()
        self.output_topic_order = safe_topics.copy()

        self.input_topic_vars: dict[str, tk.BooleanVar] = {}
        self.output_topic_vars: dict[str, tk.BooleanVar] = {}
        self.input_topic_buttons: dict[str, tk.Checkbutton] = {}
        self.output_topic_buttons: dict[str, tk.Checkbutton] = {}

        self._build_topic_checkboxes("input")
        self._build_topic_checkboxes("output")

    def _build_topic_checkboxes(self, list_type: TopicListType) -> None:
        if list_type == "input":
            frame = self.input_topic_frame.content
            topic_order = self.input_topic_order
            topic_vars = self.input_topic_vars
            topic_buttons = self.input_topic_buttons
        else:
            frame = self.output_topic_frame.content
            topic_order = self.output_topic_order
            topic_vars = self.output_topic_vars
            topic_buttons = self.output_topic_buttons

        for topic in topic_order:
            topic_var = topic_vars.get(topic, tk.BooleanVar(value=False))
            topic_vars[topic] = topic_var

            button = tk.Checkbutton(
                frame,
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
            button.pack(anchor="w")
            topic_buttons[topic] = button

    def _on_topic_clicked(self, topic: str, list_type: TopicListType) -> None:
        if list_type == "input":
            topic_order = self.input_topic_order
            topic_buttons = self.input_topic_buttons
        else:
            topic_order = self.output_topic_order
            topic_buttons = self.output_topic_buttons

        if topic not in topic_order:
            return

        topic_order.remove(topic)
        topic_order.insert(0, topic)

        for ordered_topic in topic_order:
            button = topic_buttons[ordered_topic]
            button.pack_forget()
            button.pack(anchor="w")

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
