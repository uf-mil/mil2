from __future__ import annotations

from mil_robogym.ui.components.series_plots_section import SeriesPlotsSection


class CollectedDataSection(SeriesPlotsSection):
    """Live multi-plot panel for recorded demo state features."""

    def __init__(self, parent) -> None:
        super().__init__(
            parent,
            title="Collected Data",
            section_bg="#CFCFCF",
            content_bg="#EAEAEA",
            empty_state_message="No collected data available.",
            pending_data_message="No collected data available.",
            missing_file_message="No collected data file found.",
            empty_file_message="Collected data file is empty.",
            label_formatter=lambda metric_name: metric_name,
        )
        self.container.grid(row=0, column=1, sticky="nsew")
