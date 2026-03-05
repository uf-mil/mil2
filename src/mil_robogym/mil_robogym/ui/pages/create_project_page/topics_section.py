from __future__ import annotations

import threading
import tkinter as tk
from dataclasses import dataclass
from typing import Callable, Literal

from mil_robogym.data_collection.build_tensor_spec import build_tensor_spec
from mil_robogym.data_collection.sample_input_topics import sample_topics
from mil_robogym.data_collection.types import (
    FlattenedTopic,
    RoboGymProject,
    RoboGymTensorSpec,
    SampledTopics,
)
from mil_robogym.ui.components.scrollable_frame import ScrollableFrame

TopicListType = Literal["input", "output"]


@dataclass
class SubtopicState:
    """Store subtopic UI state for one topic list type. This keeps behavior scoped to the current component.

    Args:
        frame: Scrollable frame content container for subtopic widgets.
        expanded: Mapping of topic names to expanded or collapsed state.
        field_order: Mapping of topic names to ordered sampled field names.
        numeric_fields: Mapping of topic names to numeric field-name sets.
        vars: Mapping of topic names to field selection variables.
        loading: Set of topic names currently loading sampled fields.
        load_errors: Mapping of topic names to loading error strings.
        summary_labels: Mapping of topic names to rendered summary labels.
    Returns:
        None.
    """

    frame: tk.Frame
    expanded: dict[str, bool]
    field_order: dict[str, list[str]]
    numeric_fields: dict[str, set[str]]
    vars: dict[str, dict[str, tk.BooleanVar]]
    loading: set[str]
    load_errors: dict[str, str]
    summary_labels: dict[str, tk.Label]


class TopicsSection:
    """Build and manage topics, subtopics, and tensor-spec controls. This keeps behavior scoped to the current component.

    Args:
        parent: Parent widget that hosts the topics UI.
        topics: Initial list of ROS topics to render as checkboxes.
        get_project_config: Callback returning the current project config payload.
        on_selection_change: Callback run when topic or subtopic selection changes.
        show_error: Callback used to show user-facing error text.
        hide_error: Callback used to hide any active error text.
    Returns:
        None.
    """

    def __init__(
        self,
        parent: tk.Widget,
        topics: list[str],
        get_project_config: Callable[[], RoboGymProject],
        on_selection_change: Callable[[], None],
        show_error: Callable[[str], None],
        hide_error: Callable[[], None],
    ) -> None:
        """Initialize topics widgets, state containers, and button callbacks. This keeps behavior scoped to the current component.

        Args:
            parent: Parent widget where this section is rendered.
            topics: Discovered topic names available for user selection.
            get_project_config: Callback used when computing tensor specs.
            on_selection_change: Callback used to notify form validity updates.
            show_error: Callback used to render form-level error feedback.
            hide_error: Callback used to clear form-level error feedback.
        Returns:
            None.
        """
        self._get_project_config = get_project_config
        self._on_selection_change = on_selection_change
        self._show_error = show_error
        self._hide_error = hide_error

        self._tensor_spec: RoboGymTensorSpec | None = None
        self._tensor_spec_topic_signature: (
            tuple[tuple[str, ...], tuple[str, ...]] | None
        ) = None

        input_topics_label = tk.Label(
            parent,
            text="Input Topics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 16, "bold"),
            anchor="w",
        )
        input_topics_label.grid(
            row=6,
            column=0,
            columnspan=2,
            sticky="w",
            padx=14,
            pady=(6, 2),
        )

        output_topics_label = tk.Label(
            parent,
            text="Output Topics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 16, "bold"),
            anchor="w",
        )
        output_topics_label.grid(
            row=6,
            column=3,
            columnspan=3,
            sticky="w",
            padx=(8, 14),
            pady=(6, 2),
        )

        outer = tk.Frame(parent, bg="#DADADA")
        outer.grid(row=7, column=0, columnspan=6, sticky="nsew", padx=14, pady=(0, 8))
        outer.grid_columnconfigure(0, weight=1)
        outer.grid_columnconfigure(1, weight=1)
        outer.grid_rowconfigure(0, weight=1)

        self.input_topic_frame = ScrollableFrame(outer, bg="#DADADA")
        self.input_topic_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.output_topic_frame = ScrollableFrame(outer, bg="#DADADA")
        self.output_topic_frame.grid(row=0, column=1, sticky="nsew")

        input_subtopics_label = tk.Label(
            parent,
            text="Input Subtopics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15, "bold"),
            anchor="w",
        )
        input_subtopics_label.grid(
            row=8,
            column=0,
            columnspan=2,
            sticky="w",
            padx=14,
            pady=(0, 2),
        )

        output_subtopics_label = tk.Label(
            parent,
            text="Output Subtopics",
            bg="#DADADA",
            fg="black",
            font=("Arial", 15, "bold"),
            anchor="w",
        )
        output_subtopics_label.grid(
            row=8,
            column=3,
            columnspan=3,
            sticky="w",
            padx=(8, 14),
            pady=(0, 2),
        )

        subtopic_outer = tk.Frame(parent, bg="#DADADA")
        subtopic_outer.grid(
            row=9,
            column=0,
            columnspan=6,
            sticky="nsew",
            padx=14,
            pady=(0, 8),
        )
        subtopic_outer.grid_columnconfigure(0, weight=1)
        subtopic_outer.grid_columnconfigure(1, weight=1)
        subtopic_outer.grid_rowconfigure(0, weight=1)

        self.input_subtopic_frame = ScrollableFrame(subtopic_outer, bg="#DADADA")
        self.input_subtopic_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.output_subtopic_frame = ScrollableFrame(subtopic_outer, bg="#DADADA")
        self.output_subtopic_frame.grid(row=0, column=1, sticky="nsew")

        self.compute_tensor_spec_button = tk.Button(
            parent,
            text="Compute Tensor Spec",
            command=self._on_compute_tensor_spec,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 14),
            padx=8,
            pady=5,
        )
        self.compute_tensor_spec_button.grid(
            row=10,
            column=0,
            columnspan=2,
            sticky="nsew",
            padx=(14, 8),
            pady=(2, 6),
        )

        self.tensor_spec_status_var = tk.StringVar(value="Tensor Spec: Not computed")
        self.tensor_spec_status_label = tk.Label(
            parent,
            textvariable=self.tensor_spec_status_var,
            bg="#DADADA",
            fg="#444444",
            font=("Arial", 12),
            anchor="w",
        )
        self.tensor_spec_status_label.grid(
            row=10,
            column=2,
            columnspan=4,
            sticky="nsew",
            padx=(8, 14),
            pady=(2, 6),
        )

        safe_topics = topics if topics else ["No topics found"]

        self.input_topic_vars: dict[str, tk.BooleanVar] = {}
        self.output_topic_vars: dict[str, tk.BooleanVar] = {}
        self.input_topic_buttons: dict[str, tk.Checkbutton] = {}
        self.output_topic_buttons: dict[str, tk.Checkbutton] = {}

        self.input_topic_order = safe_topics.copy()
        self.output_topic_order = safe_topics.copy()

        self.input_subtopic_vars: dict[str, dict[str, tk.BooleanVar]] = {}
        self.output_subtopic_vars: dict[str, dict[str, tk.BooleanVar]] = {}
        self.input_subtopic_field_order: dict[str, list[str]] = {}
        self.output_subtopic_field_order: dict[str, list[str]] = {}
        self.input_subtopic_numeric_fields: dict[str, set[str]] = {}
        self.output_subtopic_numeric_fields: dict[str, set[str]] = {}
        self.input_subtopic_expanded: dict[str, bool] = {}
        self.output_subtopic_expanded: dict[str, bool] = {}
        self.input_subtopic_loading: set[str] = set()
        self.output_subtopic_loading: set[str] = set()
        self.input_subtopic_load_errors: dict[str, str] = {}
        self.output_subtopic_load_errors: dict[str, str] = {}
        self.input_subtopic_summary_labels: dict[str, tk.Label] = {}
        self.output_subtopic_summary_labels: dict[str, tk.Label] = {}

        self._build_topic_checkboxes("input")
        self._build_topic_checkboxes("output")
        self._refresh_subtopic_sections("input")
        self._refresh_subtopic_sections("output")

    def _build_topic_checkboxes(self, list_type: TopicListType) -> None:
        """Build topic checkboxes for either input or output topics. This keeps behavior scoped to the current component.

        Args:
            list_type: Target topic list type, either "input" or "output".
        Returns:
            None.
        """
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

            topic_button = tk.Checkbutton(
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
            topic_button.pack(anchor="w")
            topic_buttons[topic] = topic_button

    def _on_topic_clicked(self, topic: str, list_type: TopicListType) -> None:
        """Handle topic checkbox clicks and reorder the selected topic list. This keeps behavior scoped to the current component.

        Args:
            topic: Topic that was clicked by the user.
            list_type: Topic list type where the click occurred.
        Returns:
            None.
        """
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

        self.invalidate_tensor_spec()
        self._refresh_subtopic_sections(list_type)
        self._on_selection_change()

    def _subtopic_state(self, list_type: TopicListType) -> SubtopicState:
        """Return subtopic state containers for one topic list type. This keeps behavior scoped to the current component.

        Args:
            list_type: Topic list type whose subtopic state is requested.
        Returns:
            A SubtopicState object with references to the selected state maps.
        """
        if list_type == "input":
            return SubtopicState(
                frame=self.input_subtopic_frame.content,
                expanded=self.input_subtopic_expanded,
                field_order=self.input_subtopic_field_order,
                numeric_fields=self.input_subtopic_numeric_fields,
                vars=self.input_subtopic_vars,
                loading=self.input_subtopic_loading,
                load_errors=self.input_subtopic_load_errors,
                summary_labels=self.input_subtopic_summary_labels,
            )

        return SubtopicState(
            frame=self.output_subtopic_frame.content,
            expanded=self.output_subtopic_expanded,
            field_order=self.output_subtopic_field_order,
            numeric_fields=self.output_subtopic_numeric_fields,
            vars=self.output_subtopic_vars,
            loading=self.output_subtopic_loading,
            load_errors=self.output_subtopic_load_errors,
            summary_labels=self.output_subtopic_summary_labels,
        )

    def _is_numeric_subtopic_value(self, value: object) -> bool:
        """Check whether a sampled subtopic value can become a scalar feature. This keeps behavior scoped to the current component.

        Args:
            value: Sampled flattened field value from a ROS message.
        Returns:
            True when the value is bool, int, or float; otherwise False.
        """
        return isinstance(value, (bool, int, float))

    def _sample_subtopics_for_topic(
        self,
        topic: str,
    ) -> tuple[list[str], set[str], str | None]:
        """Sample one topic and return field order, numeric fields, and an error. This keeps behavior scoped to the current component.

        Args:
            topic: Topic name to sample and flatten.
        Returns:
            A tuple of field order, numeric field names, and optional error text.
        """
        try:
            sampled: SampledTopics = sample_topics([topic])
            flattened: FlattenedTopic = sampled[topic]
        except (RuntimeError, ValueError, KeyError) as exc:
            return [], set(), str(exc) or type(exc).__name__

        field_order = list(flattened.keys())
        numeric_fields = {
            field
            for field, value in flattened.items()
            if self._is_numeric_subtopic_value(value)
        }
        return field_order, numeric_fields, None

    def _start_subtopic_load(self, topic: str, list_type: TopicListType) -> None:
        """Start asynchronous subtopic sampling for one selected topic. This keeps behavior scoped to the current component.

        Args:
            topic: Topic name whose subtopics should be loaded.
            list_type: Topic list type where the topic belongs.
        Returns:
            None.
        """
        state = self._subtopic_state(list_type)
        if topic in state.field_order or topic in state.loading:
            return

        state.loading.add(topic)
        state.load_errors.pop(topic, None)

        def _worker() -> None:
            field_order, numeric_fields, error_text = self._sample_subtopics_for_topic(
                topic,
            )
            state.frame.after(
                0,
                lambda: self._finish_subtopic_load(
                    topic,
                    list_type,
                    field_order,
                    numeric_fields,
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
        error_text: str | None,
    ) -> None:
        """Apply loaded subtopic results to Tk widgets on the main thread. This keeps behavior scoped to the current component.

        Args:
            topic: Topic that completed subtopic loading.
            list_type: Topic list type where the topic belongs.
            field_order: Ordered flattened subtopic fields.
            numeric_fields: Flattened fields considered numeric.
            error_text: Error text when loading failed, else None.
        Returns:
            None.
        """
        state = self._subtopic_state(list_type)
        state.loading.discard(topic)

        if error_text is not None:
            state.load_errors[topic] = error_text
            self._refresh_subtopic_sections(list_type)
            return

        state.load_errors.pop(topic, None)
        state.field_order[topic] = field_order
        state.numeric_fields[topic] = numeric_fields

        topic_vars = state.vars.setdefault(topic, {})
        for field in field_order:
            if field in numeric_fields and field not in topic_vars:
                topic_vars[field] = tk.BooleanVar(value=True)

        self._refresh_subtopic_sections(list_type)

    def _subtopic_summary_text(self, topic: str, list_type: TopicListType) -> str:
        """Build summary text for one topic's subtopic selection state. This keeps behavior scoped to the current component.

        Args:
            topic: Topic name whose summary text is requested.
            list_type: Topic list type where the topic belongs.
        Returns:
            Human-readable summary of selected numeric and non-numeric fields.
        """
        state = self._subtopic_state(list_type)
        field_order = state.field_order.get(topic, [])
        numeric_fields = state.numeric_fields.get(topic, set())
        topic_vars = state.vars.get(topic, {})
        is_loading = topic in state.loading
        load_error = state.load_errors.get(topic)

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
        non_numeric_count = len(field_order) - len(numeric_fields)

        summary = f"{topic} ({selected_numeric}/{len(numeric_fields)} numeric selected"
        if non_numeric_count:
            summary += f", {non_numeric_count} non-numeric"
        summary += ")"
        return summary

    def _update_subtopic_summary(self, topic: str, list_type: TopicListType) -> None:
        """Update one visible subtopic summary label if it is rendered. This keeps behavior scoped to the current component.

        Args:
            topic: Topic name whose summary label should be updated.
            list_type: Topic list type where the topic belongs.
        Returns:
            None.
        """
        state = self._subtopic_state(list_type)
        label = state.summary_labels.get(topic)
        if label is None:
            return

        label.configure(text=self._subtopic_summary_text(topic, list_type))

    def _toggle_subtopic_section(self, topic: str, list_type: TopicListType) -> None:
        """Expand or collapse one topic's subtopic section. This keeps behavior scoped to the current component.

        Args:
            topic: Topic name whose section should toggle expansion state.
            list_type: Topic list type where the topic belongs.
        Returns:
            None.
        """
        state = self._subtopic_state(list_type)
        state.expanded[topic] = not state.expanded.get(topic, False)
        self._refresh_subtopic_sections(list_type)

    def _on_subtopic_toggled(self, topic: str, list_type: TopicListType) -> None:
        """Handle subtopic checkbox toggles by invalidating cached tensor spec. This keeps behavior scoped to the current component.

        Args:
            topic: Topic name whose subtopic toggle changed.
            list_type: Topic list type where the topic belongs.
        Returns:
            None.
        """
        self.invalidate_tensor_spec()
        self._update_subtopic_summary(topic, list_type)
        self._on_selection_change()

    def _refresh_subtopic_sections(self, list_type: TopicListType) -> None:
        """Rebuild the subtopic panel for currently selected topics. This keeps behavior scoped to the current component.

        Args:
            list_type: Topic list type to refresh.
        Returns:
            None.
        """
        state = self._subtopic_state(list_type)
        frame = state.frame

        state.summary_labels.clear()
        for child in frame.winfo_children():
            child.destroy()

        selected_topics = self.selected_topics(list_type)
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

        for topic in selected_topics:
            section = tk.Frame(frame, bg="#DADADA", bd=1, relief="flat")
            section.pack(fill="x", anchor="w", pady=(0, 6))

            if topic not in state.field_order:
                self._start_subtopic_load(topic, list_type)

            header = tk.Frame(section, bg="#DADADA")
            header.pack(fill="x")

            is_expanded = state.expanded.get(topic, False)
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
            state.summary_labels[topic] = summary_label

            if not is_expanded:
                continue

            fields_container = tk.Frame(section, bg="#DADADA")
            fields_container.pack(fill="x", padx=(24, 0), pady=(4, 0))

            field_order = state.field_order.get(topic, [])
            is_loading = topic in state.loading
            load_error = state.load_errors.get(topic)
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

            numeric_fields = state.numeric_fields.get(topic, set())
            topic_vars = state.vars.setdefault(topic, {})
            if not field_order:
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
                        bg="#DADADA",
                        fg="black",
                        activebackground="#DADADA",
                        font=("Arial", 10),
                        anchor="w",
                        highlightthickness=0,
                    )
                    field_checkbox.pack(anchor="w")
                    continue

                non_numeric_label = tk.Label(
                    fields_container,
                    text=f"{field} (non-numeric)",
                    bg="#DADADA",
                    fg="#777777",
                    font=("Arial", 10),
                    anchor="w",
                )
                non_numeric_label.pack(anchor="w")

    def selected_topics(self, list_type: TopicListType) -> list[str]:
        """Return selected topics in display order for one list type. This keeps behavior scoped to the current component.

        Args:
            list_type: Topic list type to query.
        Returns:
            Ordered list of selected topic names.
        """
        if list_type == "input":
            topic_order = self.input_topic_order
            topic_vars = self.input_topic_vars
        else:
            topic_order = self.output_topic_order
            topic_vars = self.output_topic_vars

        return [topic for topic in topic_order if topic_vars[topic].get()]

    def get_selected_input_topics(self) -> list[str]:
        """Return selected input topic names in current UI order. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            Ordered list of selected input topic names.
        """
        return self.selected_topics("input")

    def get_selected_output_topics(self) -> list[str]:
        """Return selected output topic names in current UI order. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            Ordered list of selected output topic names.
        """
        return self.selected_topics("output")

    def has_valid_topic_selection(self) -> bool:
        """Check whether at least one input and output topic are selected. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            True when both input and output topic selections are non-empty.
        """
        return bool(self.get_selected_input_topics()) and bool(
            self.get_selected_output_topics(),
        )

    def _current_tensor_spec_signature(self) -> tuple[tuple[str, ...], tuple[str, ...]]:
        """Build a cache signature from current input and output topic selections. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            Tuple containing immutable selected input and output topic sequences.
        """
        return (
            tuple(self.get_selected_input_topics()),
            tuple(self.get_selected_output_topics()),
        )

    def _selected_subtopic_fields(
        self,
        list_type: TopicListType,
    ) -> dict[str, set[str]]:
        """Return selected numeric subtopic fields per selected topic. This keeps behavior scoped to the current component.

        Args:
            list_type: Topic list type whose subtopic selections are requested.
        Returns:
            Mapping from topic names to selected numeric field-name sets.
        """
        state = self._subtopic_state(list_type)
        selected_fields: dict[str, set[str]] = {}

        for topic in self.selected_topics(list_type):
            field_order = state.field_order.get(topic)
            numeric_fields = state.numeric_fields.get(topic)
            topic_vars = state.vars.get(topic, {})
            if not field_order or numeric_fields is None:
                continue

            chosen = {
                field
                for field in field_order
                if field in numeric_fields
                and (field not in topic_vars or topic_vars[field].get())
            }
            selected_fields[topic] = chosen

        return selected_fields

    def _apply_subtopic_filters_to_tensor_spec(
        self,
        tensor_spec: RoboGymTensorSpec,
    ) -> RoboGymTensorSpec:
        """Filter tensor-spec features by selected numeric subtopic fields. This keeps behavior scoped to the current component.

        Args:
            tensor_spec: Tensor specification returned by the backend builder.
        Returns:
            Tensor specification with input and output features filtered by UI choices.
        """
        filtered: RoboGymTensorSpec = {
            "input_features": list(tensor_spec["input_features"]),
            "output_features": list(tensor_spec["output_features"]),
            "input_dim": tensor_spec["input_dim"],
            "output_dim": tensor_spec["output_dim"],
            "ignored_input_features": {
                key: list(value)
                for key, value in tensor_spec["ignored_input_features"].items()
            },
            "ignored_output_features": {
                key: list(value)
                for key, value in tensor_spec["ignored_output_features"].items()
            },
        }

        input_filters = self._selected_subtopic_fields("input")
        output_filters = self._selected_subtopic_fields("output")

        def _feature_kept(feature: str, allowed_fields: dict[str, set[str]]) -> bool:
            if ":" not in feature:
                return True
            topic, field = feature.split(":", 1)
            allowed = allowed_fields.get(topic)
            if allowed is None:
                return True
            return field in allowed

        if input_filters:
            filtered["input_features"] = [
                feature
                for feature in filtered["input_features"]
                if _feature_kept(feature, input_filters)
            ]
            filtered["input_dim"] = len(filtered["input_features"])

        if output_filters:
            filtered["output_features"] = [
                feature
                for feature in filtered["output_features"]
                if _feature_kept(feature, output_filters)
            ]
            filtered["output_dim"] = len(filtered["output_features"])

        return filtered

    def _set_tensor_spec_status(self, text: str, *, fg: str) -> None:
        """Update the tensor-spec status label text and foreground color. This keeps behavior scoped to the current component.

        Args:
            text: Status text displayed next to the tensor-spec button.
            fg: Foreground color used to render the status label.
        Returns:
            None.
        """
        self.tensor_spec_status_var.set(text)
        self.tensor_spec_status_label.configure(fg=fg)

    def invalidate_tensor_spec(self) -> None:
        """Clear cached tensor-spec values after selection changes. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        self._tensor_spec = None
        self._tensor_spec_topic_signature = None
        self._set_tensor_spec_status("Tensor Spec: Not computed", fg="#444444")

    def _on_compute_tensor_spec(self) -> None:
        """Compute tensor dimensions from current topic and subtopic selections. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            None.
        """
        selected_input_topics = self.get_selected_input_topics()
        selected_output_topics = self.get_selected_output_topics()
        if not selected_input_topics or not selected_output_topics:
            self.invalidate_tensor_spec()
            self._show_error(
                "Select at least one input and output topic before computing tensor spec.",
            )
            return

        project_cfg = self._get_project_config()

        try:
            tensor_spec = build_tensor_spec(project_cfg)
        except (RuntimeError, ValueError, KeyError) as exc:
            self.invalidate_tensor_spec()
            self._set_tensor_spec_status("Tensor Spec: Failed", fg="#AA2222")
            self._show_error(str(exc) or type(exc).__name__)
            return

        tensor_spec = self._apply_subtopic_filters_to_tensor_spec(tensor_spec)
        if selected_input_topics and tensor_spec["input_dim"] == 0:
            self.invalidate_tensor_spec()
            self._set_tensor_spec_status("Tensor Spec: Failed", fg="#AA2222")
            self._show_error(
                "No numeric input subtopics selected. Expand input topics and select at least one numeric subtopic.",
            )
            return

        if selected_output_topics and tensor_spec["output_dim"] == 0:
            self.invalidate_tensor_spec()
            self._set_tensor_spec_status("Tensor Spec: Failed", fg="#AA2222")
            self._show_error(
                "No numeric output subtopics selected. Expand output topics and select at least one numeric subtopic.",
            )
            return

        self._tensor_spec = tensor_spec
        self._tensor_spec_topic_signature = self._current_tensor_spec_signature()

        ignored_input = sum(
            len(fields) for fields in tensor_spec["ignored_input_features"].values()
        )
        ignored_output = sum(
            len(fields) for fields in tensor_spec["ignored_output_features"].values()
        )
        self._set_tensor_spec_status(
            (
                "Tensor Spec: "
                f"in={tensor_spec['input_dim']} "
                f"out={tensor_spec['output_dim']} "
                f"ignored_in={ignored_input} "
                f"ignored_out={ignored_output}"
            ),
            fg="#222222",
        )
        self._hide_error()

    def get_tensor_spec_for_current_selection(self) -> RoboGymTensorSpec | None:
        """Return cached tensor spec only when it matches current selections. This keeps behavior scoped to the current component.

        Args:
            None.
        Returns:
            Matching cached tensor spec when valid, otherwise None.
        """
        if (
            self._tensor_spec is not None
            and self._tensor_spec_topic_signature
            == self._current_tensor_spec_signature()
        ):
            return self._tensor_spec

        return None
