"""Tests for shared state-feature extraction used by demo graphs and CSV writing."""

from __future__ import annotations

from pathlib import Path

import pandas as pd

from mil_robogym.data_collection.utils import extract_selected_state_features
from mil_robogym.data_collection.writers.csv_writer import AsyncCSVWriter


def test_extract_selected_state_features_preserves_requested_feature_order():
    """The flattened mapping should keep the tensor-spec feature order."""
    state = {
        "/ping": {
            "origin_distance_m": 3.5,
            "frequency": 42.0,
            "origin_direction_body": {
                "x": 0.1,
                "y": 0.2,
            },
        },
    }
    feature_names = [
        "/ping:frequency",
        "/ping:origin_direction_body.y",
        "/ping:origin_distance_m",
    ]

    features = extract_selected_state_features(state, feature_names)

    assert list(features.keys()) == feature_names
    assert features["/ping:frequency"] == 42.0
    assert features["/ping:origin_direction_body.y"] == 0.2
    assert features["/ping:origin_distance_m"] == 3.5


def test_async_csv_writer_fetch_state_series_reads_all_saved_columns(tmp_path: Path):
    """Reads the full saved numerical state CSV into ordered numeric series."""
    csv_path = tmp_path / "data.csv"
    pd.DataFrame(
        {
            "/ping:frequency": [40.0, 41.0],
            "/ping:origin_distance_m": [3.5, 3.2],
        },
    ).to_csv(csv_path, index=False)

    writer = AsyncCSVWriter.__new__(AsyncCSVWriter)
    writer.project = {
        "tensor_spec": {
            "input_features": [
                "/ping:frequency",
                "/ping:origin_distance_m",
            ],
        },
    }
    writer.numerical_state_csv = csv_path

    series = writer.fetch_state_series()

    assert series == {
        "/ping:frequency": [40.0, 41.0],
        "/ping:origin_distance_m": [3.5, 3.2],
    }
