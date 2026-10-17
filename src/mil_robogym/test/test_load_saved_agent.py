"""Tests for loading saved agent artifacts."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from mil_robogym.data_collection.load_saved_agent import (
    load_saved_agent,
    load_saved_agent_model,
)
from mil_robogym.vairl.observation_preprocessor import (
    ExternalEncoderSpec,
    ObservationPreprocessor,
)


def _write_saved_agent(
    project_dir: Path,
    *,
    agent_name: str,
    model_file_name: str = "generator_model.zip",
    checkpoint_episode: int | None = None,
    include_metrics_csv: bool = True,
    preprocessor: ObservationPreprocessor | None = None,
    preprocessor_file_name: str = "observation_preprocessor.pt",
) -> Path:
    agent_dir = project_dir / "agents" / agent_name
    agent_dir.mkdir(parents=True, exist_ok=True)

    config = {
        "robogym_agent": {
            "name": agent_name,
            "num_demos": 4,
            "model_file": model_file_name,
            "training_settings": {
                "num_episodes": 20,
                "rollout_steps": 512,
                "save_every": 5,
            },
        },
    }
    if checkpoint_episode is not None:
        config["robogym_agent"]["checkpoint_episode"] = checkpoint_episode
    if preprocessor is not None:
        config["robogym_agent"]["preprocessor_file"] = preprocessor_file_name

    (agent_dir / "config.yaml").write_text(
        yaml.safe_dump(config, sort_keys=False),
        encoding="utf-8",
    )
    (agent_dir / model_file_name).write_bytes(b"fake-model")
    if preprocessor is not None:
        preprocessor.save(str(agent_dir / preprocessor_file_name))

    if include_metrics_csv:
        (agent_dir / "training_metrics.csv").write_text(
            "episode,reward_mean\n1,0.5\n",
            encoding="utf-8",
        )

    return agent_dir


def test_load_saved_agent_reads_valid_saved_agent(tmp_path: Path, monkeypatch):
    """Returns a validated handle for a saved agent folder."""
    project_dir = tmp_path / "demo_project"
    agent_dir = _write_saved_agent(
        project_dir,
        agent_name="2026_03_30_11_15_am_ep_0005",
        checkpoint_episode=5,
    )

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent.get_training_project_dir_path",
        lambda _project: project_dir,
    )

    handle = load_saved_agent(
        {"robogym_project": {"name": "Demo Project"}},
        "2026_03_30_11_15_am_ep_0005",
    )

    assert handle.project_name == "Demo Project"
    assert handle.agent_name == "2026_03_30_11_15_am_ep_0005"
    assert handle.agent_dir == agent_dir
    assert handle.model_path == agent_dir / "generator_model.zip"
    assert handle.preprocessor_path is None
    assert handle.metrics_csv_path == agent_dir / "training_metrics.csv"
    assert handle.model_file_name == "generator_model.zip"
    assert handle.num_demos == 4
    assert handle.checkpoint_episode == 5
    assert handle.is_final is False
    assert handle.training_settings is not None
    assert handle.training_settings["num_episodes"] == 20


def test_load_saved_agent_rejects_missing_model_file(tmp_path: Path, monkeypatch):
    """Raises a clear error if the saved model file is missing."""
    project_dir = tmp_path / "demo_project"
    agent_dir = _write_saved_agent(project_dir, agent_name="saved_final")
    (agent_dir / "generator_model.zip").unlink()

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent.get_training_project_dir_path",
        lambda _project: project_dir,
    )

    try:
        load_saved_agent({"name": "Demo Project"}, "saved_final")
    except FileNotFoundError as exc:
        assert "Saved model file does not exist" in str(exc)
    else:
        raise AssertionError("Expected FileNotFoundError for missing model file.")


def test_load_saved_agent_rejects_unsafe_model_paths(tmp_path: Path, monkeypatch):
    """Rejects model_file config values that escape the agent directory."""
    project_dir = tmp_path / "demo_project"
    _write_saved_agent(
        project_dir,
        agent_name="saved_final",
        model_file_name="../outside.zip",
    )

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent.get_training_project_dir_path",
        lambda _project: project_dir,
    )

    try:
        load_saved_agent({"name": "Demo Project"}, "saved_final")
    except ValueError as exc:
        assert "must stay inside the agent folder" in str(exc)
    else:
        raise AssertionError("Expected ValueError for unsafe model path.")


def test_load_saved_agent_model_predicts_expected_output_size(
    tmp_path: Path,
    monkeypatch,
):
    """Loads a saved model and returns actions with the expected width."""
    project_dir = tmp_path / "demo_project"
    _write_saved_agent(
        project_dir,
        agent_name="saved_final",
    )

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent.get_training_project_dir_path",
        lambda _project: project_dir,
    )

    class DummyPolicy:
        def __init__(self):
            self.action_space = type("ActionSpace", (), {"shape": (4,)})()

        def predict(self, observation, deterministic=True):
            assert observation.shape == (6,)
            return np.array([0.1, 0.2, 0.3, 0.4], dtype=np.float32), None

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent._load_policy_model",
        lambda _path: DummyPolicy(),
    )

    loaded_agent = load_saved_agent_model(
        {
            "robogym_project": {
                "name": "Demo Project",
                "tensor_spec": {
                    "input_dim": 6,
                    "output_dim": 4,
                },
            },
        },
        "saved_final",
    )

    action = loaded_agent.predict(np.zeros(6, dtype=np.float32))

    assert loaded_agent.handle.agent_name == "saved_final"
    assert loaded_agent.input_size == 6
    assert loaded_agent.encoded_input_size == 6
    assert loaded_agent.output_size == 4
    assert action.shape == (4,)


def test_load_saved_agent_model_accepts_raw_abstract_inputs(
    tmp_path: Path,
    monkeypatch,
):
    """Loads a saved preprocessor bundle and encodes raw abstract inputs on predict."""
    project_dir = tmp_path / "demo_project"
    preprocessor = ObservationPreprocessor(
        numeric_feature_names=["/numeric:value"],
        encoder_specs=[
            ExternalEncoderSpec(
                feature_name="/camera",
                data_type="image",
                ros_type="sensor_msgs/msg/Image",
                output_dim=8,
                resize_shape=(8, 8),
            ),
        ],
    )
    agent_dir = _write_saved_agent(
        project_dir,
        agent_name="saved_with_preprocessor",
        preprocessor=preprocessor,
    )

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent.get_training_project_dir_path",
        lambda _project: project_dir,
    )

    class DummyPolicy:
        def __init__(self):
            self.observation_space = type("ObservationSpace", (), {"shape": (9,)})()
            self.action_space = type("ActionSpace", (), {"shape": (4,)})()

        def predict(self, observation, deterministic=True):
            assert observation.shape == (9,)
            assert observation.dtype == np.float32
            return np.array([0.4, 0.3, 0.2, 0.1], dtype=np.float32), None

    monkeypatch.setattr(
        "mil_robogym.data_collection.load_saved_agent._load_policy_model",
        lambda _path: DummyPolicy(),
    )

    loaded_agent = load_saved_agent_model(
        {
            "robogym_project": {
                "name": "Demo Project",
                "tensor_spec": {
                    "input_dim": 1,
                    "output_dim": 4,
                },
            },
        },
        "saved_with_preprocessor",
    )

    action = loaded_agent.predict(
        [
            0.25,
            np.zeros((8, 8, 3), dtype=np.uint8),
        ],
    )

    assert loaded_agent.handle.agent_dir == agent_dir
    assert loaded_agent.handle.preprocessor_path == (
        agent_dir / "observation_preprocessor.pt"
    )
    assert loaded_agent.input_size == 2
    assert loaded_agent.encoded_input_size == 9
    assert loaded_agent.output_size == 4
    assert action.shape == (4,)
