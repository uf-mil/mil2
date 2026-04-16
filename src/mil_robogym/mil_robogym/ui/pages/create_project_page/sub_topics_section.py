from __future__ import annotations

import threading
import tkinter as tk
from typing import Callable, Literal

from mil_robogym.data_collection.sample_input_topics import (
    resolve_topic_message_types,
    sample_topics,
)
from mil_robogym.data_collection.topic_schema import (
    SupportedNonNumericFieldOption,
    is_image_message_type,
    supported_non_numeric_fields_for_message_type,
)
from mil_robogym.data_collection.types import (
    FlattenedTopic,
    NonNumericTopicFieldSelection,
    SampledTopics,
)
from mil_robogym.data_collection.utils import (
    filter_populated_non_numeric_topic_fields,
)
from mil_robogym.ui.components.scrollable_frame import ScrollableFrame

TopicListType = Literal["input", "output"]


class SubTopicsSection:
    """Render and manage input/output subtopic selection panels."""

    def __init__(
        self,
        parent: tk.Widget,
        on_selection_change: Callable[[], None],
    ) -> None:
        self._on_selection_change = on_selection_change
        self._selection_enabled = True

        self._selected_input_topics: list[str] = []
        self._selected_output_topics: list[str] = []

        self.input_subtopics_label = tk.Label(
            parent,
            text="Input Subtopics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 16, "bold"),
            anchor="w",
        )
        self.input_subtopics_label.grid(
            row=8,
            column=0,
            columnspan=2,
            sticky="w",
            padx=14,
            pady=(0, 2),
        )

        self.output_subtopics_label = tk.Label(
            parent,
            text="Output Subtopics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 16, "bold"),
            anchor="w",
        )
        self.output_subtopics_label.grid(
            row=8,
            column=3,
            columnspan=3,
            sticky="w",
            padx=(8, 14),
            pady=(0, 2),
        )

        self.subtopic_outer = tk.Frame(parent, bg="#DADADA")
        self.subtopic_outer.grid(
            row=9,
            column=0,
            columnspan=6,
            sticky="nsew",
            padx=14,
            pady=(0, 6),
        )
        self.subtopic_outer.grid_columnconfigure(0, weight=1)
        self.subtopic_outer.grid_columnconfigure(1, weight=1)
        self.subtopic_outer.grid_rowconfigure(0, weight=1)

        self.input_subtopic_frame = ScrollableFrame(self.subtopic_outer, bg="#DADADA")
        self.input_subtopic_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.output_subtopic_frame = ScrollableFrame(self.subtopic_outer, bg="#DADADA")
        self.output_subtopic_frame.grid(row=0, column=1, sticky="nsew")

        self.input_subtopic_vars: dict[str, dict[str, tk.BooleanVar]] = {}
        self.output_subtopic_vars: dict[str, dict[str, tk.BooleanVar]] = {}
        self.input_non_numeric_subtopic_vars: dict[str, dict[str, tk.BooleanVar]] = {}
        self.output_non_numeric_subtopic_vars: dict[str, dict[str, tk.BooleanVar]] = {}
        self.input_subtopic_field_order: dict[str, list[str]] = {}
        self.output_subtopic_field_order: dict[str, list[str]] = {}
        self.input_subtopic_numeric_fields: dict[str, set[str]] = {}
        self.output_subtopic_numeric_fields: dict[str, set[str]] = {}
        self.input_subtopic_non_numeric_fields: dict[
            str,
            list[SupportedNonNumericFieldOption],
        ] = {}
        self.output_subtopic_non_numeric_fields: dict[
            str,
            list[SupportedNonNumericFieldOption],
        ] = {}
        self.input_subtopic_expanded: dict[str, bool] = {}
        self.output_subtopic_expanded: dict[str, bool] = {}
        self.input_subtopic_loading: set[str] = set()
        self.output_subtopic_loading: set[str] = set()
        self.input_subtopic_load_errors: dict[str, str] = {}
        self.output_subtopic_load_errors: dict[str, str] = {}
        self.input_subtopic_summary_labels: dict[str, tk.Label] = {}
        self.output_subtopic_summary_labels: dict[str, tk.Label] = {}
        self._desired_input_numeric_fields: dict[str, set[str]] = {}
        self._desired_output_numeric_fields: dict[str, set[str]] = {}
        self._desired_input_non_numeric_fields: dict[str, set[str]] = {}
        self._desired_output_non_numeric_fields: dict[str, set[str]] = {}

        self._refresh_subtopic_sections("input")
        self._refresh_subtopic_sections("output")

    def _subtopic_control_state(self) -> str:
        return tk.NORMAL if self._selection_enabled else tk.DISABLED

    def set_selection_enabled(self, enabled: bool) -> None:
        self._selection_enabled = enabled
        self._refresh_subtopic_sections("input")
        self._refresh_subtopic_sections("output")

    def set_selected_topics(
        self,
        *,
        input_topics: list[str],
        output_topics: list[str],
    ) -> None:
        self._selected_input_topics = list(input_topics)
        self._selected_output_topics = list(output_topics)
        self._refresh_subtopic_sections("input")
        self._refresh_subtopic_sections("output")

    def set_selected_fields(
        self,
        *,
        input_numeric_fields: dict[str, list[str]] | None = None,
        output_numeric_fields: dict[str, list[str]] | None = None,
        input_non_numeric_fields: (
            dict[str, list[NonNumericTopicFieldSelection]] | None
        ) = None,
        output_non_numeric_fields: (
            dict[str, list[NonNumericTopicFieldSelection]] | None
        ) = None,
    ) -> None:
        self._desired_input_numeric_fields = {
            topic: set(fields) for topic, fields in (input_numeric_fields or {}).items()
        }
        self._desired_output_numeric_fields = {
            topic: set(fields)
            for topic, fields in (output_numeric_fields or {}).items()
        }
        self._desired_input_non_numeric_fields = {
            topic: {field["field_path"] for field in fields}
            for topic, fields in (input_non_numeric_fields or {}).items()
        }
        self._desired_output_non_numeric_fields = {
            topic: {field["field_path"] for field in fields}
            for topic, fields in (output_non_numeric_fields or {}).items()
        }

        self._apply_desired_selection_to_loaded_topics("input")
        self._apply_desired_selection_to_loaded_topics("output")
        self._refresh_subtopic_sections("input")
        self._refresh_subtopic_sections("output")

    def reload_selected_topics(self) -> None:
        for list_type in ("input", "output"):
            field_order_map = self._subtopic_field_order_map(list_type)
            numeric_fields_map = self._subtopic_numeric_fields_map(list_type)
            non_numeric_fields_map = self._subtopic_non_numeric_fields_map(list_type)
            load_errors_map = self._subtopic_load_errors_map(list_type)

            for topic in self._selected_topics(list_type):
                field_order_map.pop(topic, None)
                numeric_fields_map.pop(topic, None)
                non_numeric_fields_map.pop(topic, None)
                load_errors_map.pop(topic, None)

            self._refresh_subtopic_sections(list_type)

    def _selected_topics(self, list_type: TopicListType) -> list[str]:
        if list_type == "input":
            return self._selected_input_topics
        return self._selected_output_topics

    def _subtopic_frame(self, list_type: TopicListType) -> tk.Frame:
        if list_type == "input":
            return self.input_subtopic_frame.content
        return self.output_subtopic_frame.content

    def _subtopic_expanded_map(self, list_type: TopicListType) -> dict[str, bool]:
        if list_type == "input":
            return self.input_subtopic_expanded
        return self.output_subtopic_expanded

    def _subtopic_field_order_map(
        self,
        list_type: TopicListType,
    ) -> dict[str, list[str]]:
        if list_type == "input":
            return self.input_subtopic_field_order
        return self.output_subtopic_field_order

    def _subtopic_numeric_fields_map(
        self,
        list_type: TopicListType,
    ) -> dict[str, set[str]]:
        if list_type == "input":
            return self.input_subtopic_numeric_fields
        return self.output_subtopic_numeric_fields

    def _subtopic_non_numeric_fields_map(
        self,
        list_type: TopicListType,
    ) -> dict[str, list[SupportedNonNumericFieldOption]]:
        if list_type == "input":
            return self.input_subtopic_non_numeric_fields
        return self.output_subtopic_non_numeric_fields

    def _subtopic_vars_map(
        self,
        list_type: TopicListType,
    ) -> dict[str, dict[str, tk.BooleanVar]]:
        if list_type == "input":
            return self.input_subtopic_vars
        return self.output_subtopic_vars

    def _non_numeric_subtopic_vars_map(
        self,
        list_type: TopicListType,
    ) -> dict[str, dict[str, tk.BooleanVar]]:
        if list_type == "input":
            return self.input_non_numeric_subtopic_vars
        return self.output_non_numeric_subtopic_vars

    def _subtopic_loading_set(self, list_type: TopicListType) -> set[str]:
        if list_type == "input":
            return self.input_subtopic_loading
        return self.output_subtopic_loading

    def _subtopic_load_errors_map(self, list_type: TopicListType) -> dict[str, str]:
        if list_type == "input":
            return self.input_subtopic_load_errors
        return self.output_subtopic_load_errors

    def _subtopic_summary_labels_map(
        self,
        list_type: TopicListType,
    ) -> dict[str, tk.Label]:
        if list_type == "input":
            return self.input_subtopic_summary_labels
        return self.output_subtopic_summary_labels

    def _desired_numeric_fields_map(
        self,
        list_type: TopicListType,
    ) -> dict[str, set[str]]:
        if list_type == "input":
            return self._desired_input_numeric_fields
        return self._desired_output_numeric_fields

    def _desired_non_numeric_fields_map(
        self,
        list_type: TopicListType,
    ) -> dict[str, set[str]]:
        if list_type == "input":
            return self._desired_input_non_numeric_fields
        return self._desired_output_non_numeric_fields

    def _is_numeric_subtopic_value(self, value: object) -> bool:
        return isinstance(value, (bool, int, float))

    def _sample_subtopics_for_topic(
        self,
        topic: str,
    ) -> tuple[
        list[str],
        set[str],
        list[SupportedNonNumericFieldOption],
        str | None,
    ]:
        try:
            topic_message_types = resolve_topic_message_types([topic])
            topic_message_type = topic_message_types[topic]
            sampled: SampledTopics = sample_topics([topic])
            flattened: FlattenedTopic = sampled[topic]
            non_numeric_fields = supported_non_numeric_fields_for_message_type(
                topic_message_type,
            )
        except (RuntimeError, ValueError, KeyError) as exc:
            return [], set(), [], str(exc) or type(exc).__name__

        if is_image_message_type(topic_message_type):
            field_order: list[str] = []
            numeric_fields: set[str] = set()
        else:
            field_order = list(flattened.keys())
            numeric_fields = {
                field
                for field, value in flattened.items()
                if self._is_numeric_subtopic_value(value)
            }
        return field_order, numeric_fields, non_numeric_fields, None

    def _start_subtopic_load(self, topic: str, list_type: TopicListType) -> None:
        field_order_map = self._subtopic_field_order_map(list_type)
        loading_set = self._subtopic_loading_set(list_type)
        load_errors_map = self._subtopic_load_errors_map(list_type)

        if topic in field_order_map or topic in loading_set:
            return

        loading_set.add(topic)
        load_errors_map.pop(topic, None)

        frame = self._subtopic_frame(list_type)

        def _worker() -> None:
            field_order, numeric_fields, non_numeric_fields, error_text = (
                self._sample_subtopics_for_topic(topic)
            )
            frame.after(
                0,
                lambda: self._finish_subtopic_load(
                    topic,
                    list_type,
                    field_order,
                    numeric_fields,
                    non_numeric_fields,
                    error_text,
                ),
            )

        threading.Thread(target=_worker, daemon=True).start()

    def _finish_subtopic_load(
        self,
        topic: str,
        list_type: TopicListType,
        field_order: list[str],
        numeric_fields: set[str],
        non_numeric_fields: list[SupportedNonNumericFieldOption],
        error_text: str | None,
    ) -> None:
        loading_set = self._subtopic_loading_set(list_type)
        load_errors_map = self._subtopic_load_errors_map(list_type)

        loading_set.discard(topic)

        if error_text is not None:
            load_errors_map[topic] = error_text
            self._refresh_subtopic_sections(list_type)
            self._on_selection_change()
            return

        load_errors_map.pop(topic, None)
        self._apply_loaded_topic_schema(
            topic,
            list_type,
            field_order,
            numeric_fields,
            non_numeric_fields,
        )
        self._refresh_subtopic_sections(list_type)
        self._on_selection_change()

    def _apply_loaded_topic_schema(
        self,
        topic: str,
        list_type: TopicListType,
        field_order: list[str],
        numeric_fields: set[str],
        non_numeric_fields: list[SupportedNonNumericFieldOption],
    ) -> None:
        field_order_map = self._subtopic_field_order_map(list_type)
        numeric_fields_map = self._subtopic_numeric_fields_map(list_type)
        non_numeric_fields_map = self._subtopic_non_numeric_fields_map(list_type)
        numeric_vars_map = self._subtopic_vars_map(list_type)
        non_numeric_vars_map = self._non_numeric_subtopic_vars_map(list_type)

        field_order_map[topic] = field_order
        numeric_fields_map[topic] = numeric_fields
        non_numeric_fields_map[topic] = non_numeric_fields

        numeric_topic_vars = numeric_vars_map.setdefault(topic, {})
        for field in list(numeric_topic_vars):
            if field not in numeric_fields:
                numeric_topic_vars.pop(field)
        for field in field_order:
            if field in numeric_fields and field not in numeric_topic_vars:
                numeric_topic_vars[field] = tk.BooleanVar(value=True)

        non_numeric_topic_vars = non_numeric_vars_map.setdefault(topic, {})
        valid_non_numeric_fields = {field.field_path for field in non_numeric_fields}
        for field in list(non_numeric_topic_vars):
            if field not in valid_non_numeric_fields:
                non_numeric_topic_vars.pop(field)
        for field in non_numeric_fields:
            if field.field_path not in non_numeric_topic_vars:
                non_numeric_topic_vars[field.field_path] = tk.BooleanVar(value=True)

        desired_numeric_fields = self._desired_numeric_fields_map(list_type).get(topic)
        if desired_numeric_fields is not None:
            for field, var in numeric_topic_vars.items():
                var.set(field in desired_numeric_fields)

        desired_non_numeric_fields = self._desired_non_numeric_fields_map(
            list_type,
        ).get(topic)
        if desired_non_numeric_fields is not None:
            for field_path, var in non_numeric_topic_vars.items():
                var.set(field_path in desired_non_numeric_fields)

    def _subtopic_summary_text(self, topic: str, list_type: TopicListType) -> str:
        field_order_map = self._subtopic_field_order_map(list_type)
        numeric_fields_map = self._subtopic_numeric_fields_map(list_type)
        non_numeric_fields_map = self._subtopic_non_numeric_fields_map(list_type)
        vars_map = self._subtopic_vars_map(list_type)
        non_numeric_vars_map = self._non_numeric_subtopic_vars_map(list_type)
        loading_set = self._subtopic_loading_set(list_type)
        load_errors_map = self._subtopic_load_errors_map(list_type)

        field_order = field_order_map.get(topic, [])
        numeric_fields = numeric_fields_map.get(topic, set())
        non_numeric_fields = non_numeric_fields_map.get(topic, [])
        topic_vars = vars_map.get(topic, {})
        non_numeric_topic_vars = non_numeric_vars_map.get(topic, {})
        is_loading = topic in loading_set
        load_error = load_errors_map.get(topic)

        if is_loading and not field_order:
            return f"{topic} (loading subtopics...)"
        if load_error and not field_order:
            return f"{topic} (failed to load subtopics)"

        selected_numeric = sum(
            1
            for field in field_order
            if field in numeric_fields
            and (field not in topic_vars or topic_vars[field].get())
        )
        selected_non_numeric = sum(
            1
            for field in non_numeric_fields
            if field.field_path not in non_numeric_topic_vars
            or non_numeric_topic_vars[field.field_path].get()
        )
        unsupported_non_numeric_count = (
            len(field_order)
            - len(numeric_fields)
            - len({field.field_path for field in non_numeric_fields})
        )

        summary = f"{topic} ({selected_numeric}/{len(numeric_fields)} numeric selected"
        if non_numeric_fields:
            summary += (
                f", {selected_non_numeric}/{len(non_numeric_fields)} special selected"
            )
        if unsupported_non_numeric_count > 0:
            summary += f", {unsupported_non_numeric_count} unsupported"
        summary += ")"
        return summary

    def _update_subtopic_summary(self, topic: str, list_type: TopicListType) -> None:
        summary_labels_map = self._subtopic_summary_labels_map(list_type)
        label = summary_labels_map.get(topic)
        if label is None:
            return

        label.configure(text=self._subtopic_summary_text(topic, list_type))

    def _toggle_subtopic_section(self, topic: str, list_type: TopicListType) -> None:
        expanded_map = self._subtopic_expanded_map(list_type)
        expanded_map[topic] = not expanded_map.get(topic, False)
        self._refresh_subtopic_sections(list_type)

    def _on_subtopic_toggled(self, topic: str, list_type: TopicListType) -> None:
        if not self._selection_enabled:
            return
        self._update_subtopic_summary(topic, list_type)
        self._on_selection_change()

    def _apply_desired_selection_to_loaded_topics(
        self,
        list_type: TopicListType,
    ) -> None:
        numeric_vars_map = self._subtopic_vars_map(list_type)
        non_numeric_vars_map = self._non_numeric_subtopic_vars_map(list_type)

        for topic, desired_fields in self._desired_numeric_fields_map(
            list_type,
        ).items():
            for field, var in numeric_vars_map.get(topic, {}).items():
                var.set(field in desired_fields)

        for topic, desired_fields in self._desired_non_numeric_fields_map(
            list_type,
        ).items():
            for field_path, var in non_numeric_vars_map.get(topic, {}).items():
                var.set(field_path in desired_fields)

    def _special_field_display_text(
        self,
        field: SupportedNonNumericFieldOption,
    ) -> str:
        if field.data_type == "image":
            return f"{field.label} image"
        return f"{field.label} (unordered set)"

    def _refresh_subtopic_sections(self, list_type: TopicListType) -> None:
        frame = self._subtopic_frame(list_type)
        selected_topics = self._selected_topics(list_type)

        summary_labels_map = self._subtopic_summary_labels_map(list_type)
        summary_labels_map.clear()

        for child in frame.winfo_children():
            child.destroy()

        if not selected_topics:
            empty_state_label = tk.Label(
                frame,
                text="Select a topic to expand subtopics.",
                bg="#DADADA",
                fg="#666666",
                font=("Arial", 11),
                anchor="w",
            )
            empty_state_label.pack(anchor="w")
            return

        field_order_map = self._subtopic_field_order_map(list_type)
        expanded_map = self._subtopic_expanded_map(list_type)
        loading_set = self._subtopic_loading_set(list_type)
        load_errors_map = self._subtopic_load_errors_map(list_type)
        numeric_fields_map = self._subtopic_numeric_fields_map(list_type)
        non_numeric_fields_map = self._subtopic_non_numeric_fields_map(list_type)
        vars_map = self._subtopic_vars_map(list_type)
        non_numeric_vars_map = self._non_numeric_subtopic_vars_map(list_type)

        for topic in selected_topics:
            section = tk.Frame(frame, bg="#DADADA", bd=1, relief="flat")
            section.pack(fill="x", anchor="w", pady=(0, 6))

            if topic not in field_order_map:
                self._start_subtopic_load(topic, list_type)

            header = tk.Frame(section, bg="#DADADA")
            header.pack(fill="x")

            is_expanded = expanded_map.get(topic, False)
            toggle = tk.Button(
                header,
                text="▼" if is_expanded else "▶",
                command=lambda t=topic, lt=list_type: self._toggle_subtopic_section(
                    t,
                    lt,
                ),
                bg="#ECECEC",
                activebackground="#DFDFDF",
                fg="black",
                relief="solid",
                bd=1,
                font=("Arial", 9, "bold"),
                width=2,
                padx=0,
                pady=0,
            )
            toggle.pack(side="left", padx=(0, 6))

            summary_label = tk.Label(
                header,
                text=self._subtopic_summary_text(topic, list_type),
                bg="#DADADA",
                fg="black",
                font=("Arial", 11),
                anchor="w",
            )
            summary_label.pack(side="left", fill="x", expand=True)
            summary_labels_map[topic] = summary_label

            if not is_expanded:
                continue

            fields_container = tk.Frame(section, bg="#DADADA")
            fields_container.pack(fill="x", padx=(24, 0), pady=(4, 0))

            field_order = field_order_map.get(topic, [])
            is_loading = topic in loading_set
            load_error = load_errors_map.get(topic)
            if load_error is not None:
                load_error_label = tk.Label(
                    fields_container,
                    text=f"Failed to load subtopics: {load_error}",
                    bg="#DADADA",
                    fg="#AA2222",
                    font=("Arial", 10),
                    anchor="w",
                )
                load_error_label.pack(anchor="w")
                continue

            if is_loading and not field_order:
                loading_label = tk.Label(
                    fields_container,
                    text="Loading subtopics...",
                    bg="#DADADA",
                    fg="#666666",
                    font=("Arial", 10),
                    anchor="w",
                )
                loading_label.pack(anchor="w")
                continue

            non_numeric_fields = non_numeric_fields_map.get(topic, [])
            if not field_order and not non_numeric_fields:
                no_subtopics_label = tk.Label(
                    fields_container,
                    text="No subtopics available.",
                    bg="#DADADA",
                    fg="#666666",
                    font=("Arial", 10),
                    anchor="w",
                )
                no_subtopics_label.pack(anchor="w")
                continue

            numeric_fields = numeric_fields_map.get(topic, set())
            topic_vars = vars_map.setdefault(topic, {})
            non_numeric_topic_vars = non_numeric_vars_map.setdefault(topic, {})
            special_fields_by_path = {
                field.field_path: field for field in non_numeric_fields
            }

            for field in field_order:
                if field in numeric_fields:
                    field_var = topic_vars.setdefault(field, tk.BooleanVar(value=True))
                    field_checkbox = tk.Checkbutton(
                        fields_container,
                        text=field,
                        variable=field_var,
                        command=lambda t=topic, lt=list_type: self._on_subtopic_toggled(
                            t,
                            lt,
                        ),
                        state=self._subtopic_control_state(),
                        bg="#DADADA",
                        fg="black",
                        activebackground="#DADADA",
                        disabledforeground="#666666",
                        font=("Arial", 10),
                        anchor="w",
                        highlightthickness=0,
                    )
                    field_checkbox.pack(anchor="w")
                    continue

                special_field = special_fields_by_path.get(field)
                if special_field is not None:
                    field_var = non_numeric_topic_vars.setdefault(
                        field,
                        tk.BooleanVar(value=True),
                    )
                    field_checkbox = tk.Checkbutton(
                        fields_container,
                        text=self._special_field_display_text(special_field),
                        variable=field_var,
                        command=lambda t=topic, lt=list_type: self._on_subtopic_toggled(
                            t,
                            lt,
                        ),
                        state=self._subtopic_control_state(),
                        bg="#DADADA",
                        fg="#1E4B7A",
                        activebackground="#DADADA",
                        disabledforeground="#5A7694",
                        font=("Arial", 10),
                        anchor="w",
                        highlightthickness=0,
                    )
                    field_checkbox.pack(anchor="w")
                    continue

                unsupported_label = tk.Label(
                    fields_container,
                    text=f"{field} (unsupported)",
                    bg="#DADADA",
                    fg="#777777",
                    font=("Arial", 10),
                    anchor="w",
                )
                unsupported_label.pack(anchor="w")

            for field in non_numeric_fields:
                if field.field_path in field_order:
                    continue
                field_var = non_numeric_topic_vars.setdefault(
                    field.field_path,
                    tk.BooleanVar(value=True),
                )
                field_checkbox = tk.Checkbutton(
                    fields_container,
                    text=self._special_field_display_text(field),
                    variable=field_var,
                    command=lambda t=topic, lt=list_type: self._on_subtopic_toggled(
                        t,
                        lt,
                    ),
                    state=self._subtopic_control_state(),
                    bg="#DADADA",
                    fg="#1E4B7A",
                    activebackground="#DADADA",
                    disabledforeground="#5A7694",
                    font=("Arial", 10),
                    anchor="w",
                    highlightthickness=0,
                )
                field_checkbox.pack(anchor="w")

    def _selected_topic_subtopics(
        self,
        list_type: TopicListType,
        *,
        ensure_loaded: bool = False,
    ) -> dict[str, list[str]]:
        selected_topic_subtopics: dict[str, list[str]] = {}

        topics = self._selected_topics(list_type)
        field_order_map = self._subtopic_field_order_map(list_type)
        numeric_fields_map = self._subtopic_numeric_fields_map(list_type)
        vars_map = self._subtopic_vars_map(list_type)
        load_errors_map = self._subtopic_load_errors_map(list_type)

        for topic in topics:
            if ensure_loaded and topic not in field_order_map:
                field_order, numeric_fields, non_numeric_fields, error_text = (
                    self._sample_subtopics_for_topic(topic)
                )
                if error_text is not None:
                    load_errors_map[topic] = error_text
                    selected_topic_subtopics[topic] = []
                    continue

                load_errors_map.pop(topic, None)
                self._apply_loaded_topic_schema(
                    topic,
                    list_type,
                    field_order,
                    numeric_fields,
                    non_numeric_fields,
                )

            field_order = field_order_map.get(topic, [])
            numeric_fields = numeric_fields_map.get(topic, set())
            topic_vars = vars_map.setdefault(topic, {})
            selected_topic_subtopics[topic] = [
                field
                for field in field_order
                if field in numeric_fields
                and (field not in topic_vars or topic_vars[field].get())
            ]

        return selected_topic_subtopics

    def _selected_non_numeric_topic_fields(
        self,
        list_type: TopicListType,
        *,
        ensure_loaded: bool = False,
    ) -> dict[str, list[NonNumericTopicFieldSelection]]:
        selected_topic_fields: dict[str, list[NonNumericTopicFieldSelection]] = {}

        topics = self._selected_topics(list_type)
        field_order_map = self._subtopic_field_order_map(list_type)
        non_numeric_fields_map = self._subtopic_non_numeric_fields_map(list_type)
        non_numeric_vars_map = self._non_numeric_subtopic_vars_map(list_type)
        load_errors_map = self._subtopic_load_errors_map(list_type)

        for topic in topics:
            if ensure_loaded and topic not in field_order_map:
                field_order, numeric_fields, non_numeric_fields, error_text = (
                    self._sample_subtopics_for_topic(topic)
                )
                if error_text is not None:
                    load_errors_map[topic] = error_text
                    continue

                load_errors_map.pop(topic, None)
                self._apply_loaded_topic_schema(
                    topic,
                    list_type,
                    field_order,
                    numeric_fields,
                    non_numeric_fields,
                )

            available_fields = non_numeric_fields_map.get(topic, [])
            non_numeric_topic_vars = non_numeric_vars_map.setdefault(topic, {})
            selected_fields = [
                {
                    "field_path": field.field_path,
                    "data_type": field.data_type,
                    "ros_type": field.ros_type,
                }
                for field in available_fields
                if field.field_path not in non_numeric_topic_vars
                or non_numeric_topic_vars[field.field_path].get()
            ]
            if selected_fields:
                selected_topic_fields[topic] = selected_fields

        return filter_populated_non_numeric_topic_fields(selected_topic_fields)

    def get_selected_input_topic_subtopics(
        self,
        *,
        ensure_loaded: bool = False,
    ) -> dict[str, list[str]]:
        return self._selected_topic_subtopics("input", ensure_loaded=ensure_loaded)

    def get_selected_output_topic_subtopics(
        self,
        *,
        ensure_loaded: bool = False,
    ) -> dict[str, list[str]]:
        return self._selected_topic_subtopics("output", ensure_loaded=ensure_loaded)

    def get_selected_input_non_numeric_topic_fields(
        self,
        *,
        ensure_loaded: bool = False,
    ) -> dict[str, list[NonNumericTopicFieldSelection]]:
        return self._selected_non_numeric_topic_fields(
            "input",
            ensure_loaded=ensure_loaded,
        )

    def get_selected_output_non_numeric_topic_fields(
        self,
        *,
        ensure_loaded: bool = False,
    ) -> dict[str, list[NonNumericTopicFieldSelection]]:
        return self._selected_non_numeric_topic_fields(
            "output",
            ensure_loaded=ensure_loaded,
        )
