"""Tests for shared state-feature extraction and CSV-backed series loading."""

from __future__ import annotations

from pathlib import Path

import pandas as pd

from mil_robogym.data_collection.utils import extract_selected_state_features
from mil_robogym.data_collection.writers.csv_writer import AsyncCSVWriter


def test_extract_selected_state_features_preserves_requested_feature_order() -> None:
    """Selected features should be returned in tensor-spec order."""
    state = {
        "/ping": {
            "frequency": 12.5,
            "confidence": 0.7,
        },
        "/pose": {
            "x": 1.2,
        },
    }

    extracted = extract_selected_state_features(
        state,
        [
            "/pose:x",
            "/ping:frequency",
        ],
    )

    assert list(extracted.keys()) == ["/pose:x", "/ping:frequency"]
    assert extracted["/pose:x"] == 1.2
    assert extracted["/ping:frequency"] == 12.5


def test_async_csv_writer_fetch_state_series_reads_all_saved_columns(
    tmp_path: Path,
) -> None:
    """State-series loading returns all requested CSV columns in project order."""
    writer = AsyncCSVWriter.__new__(AsyncCSVWriter)
    writer.project = {
        "tensor_spec": {
            "input_features": [
                "/pose:x",
                "/ping:frequency",
            ],
        },
    }
    writer.numerical_state_csv = tmp_path / "data.csv"

    pd.DataFrame(
        {
            "/ping:frequency": [11.0, 12.0],
            "/pose:x": [1.0, 2.0],
            "other": [99.0, 100.0],
        },
    ).to_csv(writer.numerical_state_csv, index=False)

    assert writer.fetch_state_series() == {
        "/pose:x": [1.0, 2.0],
        "/ping:frequency": [11.0, 12.0],
    }
