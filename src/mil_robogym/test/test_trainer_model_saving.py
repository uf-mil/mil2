"""Tests for generator model checkpoint saving during training."""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path

import yaml


class DummyTRPO:
    """Minimal generator stub that writes a model artifact to disk."""

    def __init__(self, *args, **kwargs):
        self.saved_paths: list[Path] = []

    def save(self, path: str) -> None:
        artifact_path = Path(path)
        artifact_path.write_bytes(b"generator-artifact")
        self.saved_paths.append(artifact_path)


class DummyRewardNet:
    """Minimal reward network stub for the trainer loop."""

    def __init__(self, *args, **kwargs):
        pass

    def eval(self) -> None:
        pass

    def train(self) -> None:
        pass


class DummyVAIRL:
    """Minimal VAIRL stub that returns deterministic discriminator metrics."""

    def __init__(self, *args, **kwargs):
        self.beta = 0.0

    def train_gen(self, total_timesteps: int) -> None:
        self.last_total_timesteps = total_timesteps

    def train_disc(self, *, expert_samples=None, gen_samples=None) -> dict[str, float]:
        return {"loss": 1.5, "kl": 0.25, "beta": 0.05}


def _install_trainer_import_stubs(monkeypatch) -> None:
    """Stub optional runtime dependencies so the trainer module can be imported."""
    rollout_module = types.ModuleType("imitation.data.rollout")
    rollout_module.flatten_trajectories = lambda trajectories: trajectories

    data_module = types.ModuleType("imitation.data")
    data_module.rollout = rollout_module

    networks_module = types.ModuleType("imitation.util.networks")
    networks_module.RunningNorm = object

    util_module = types.ModuleType("imitation.util.util")
    util_module.make_vec_env = lambda *args, **kwargs: types.SimpleNamespace(
        observation_space="obs_space",
        action_space="action_space",
    )

    imitation_module = types.ModuleType("imitation")
    imitation_module.data = data_module

    sb3_module = types.ModuleType("sb3_contrib")
    sb3_module.TRPO = DummyTRPO

    controller_client_module = types.ModuleType(
        "mil_robogym.clients.controller_client",
    )
    controller_client_module.ControllerClient = type("ControllerClient", (), {})

    data_collector_client_module = types.ModuleType(
        "mil_robogym.clients.data_collector_client",
    )
    data_collector_client_module.DataCollectorClient = type(
        "DataCollectorClient",
        (),
        {},
    )

    localization_client_module = types.ModuleType(
        "mil_robogym.clients.localization_client",
    )
    localization_client_module.LocalizationClient = type(
        "LocalizationClient",
        (),
        {},
    )

    world_control_client_module = types.ModuleType(
        "mil_robogym.clients.world_control_client",
    )
    world_control_client_module.WorldControlClient = type(
        "WorldControlClient",
        (),
        {},
    )

    move_client_module = types.ModuleType("mil_robogym.clients.move_client")
    move_client_module.MoveClient = type("MoveClient", (), {})

    set_pose_client_module = types.ModuleType("mil_robogym.clients.set_pose_client")
    set_pose_client_module.SetPoseClient = type("SetPoseClient", (), {})

    environment_module = types.ModuleType("mil_robogym.vairl.environment")
    environment_module.GYMNASIUM = True
    environment_module.Environment = type("Environment", (), {})

    reward_net_module = types.ModuleType("mil_robogym.vairl.reward_net")
    reward_net_module.VAIRLRewardNet = DummyRewardNet

    utils_module = types.ModuleType("mil_robogym.vairl.utils")
    utils_module.fetch_demo_trajectories = lambda *args, **kwargs: []
    utils_module.trajectories_to_batches = lambda *args, **kwargs: {
        "obs": [],
        "acts": [],
        "next_obs": [],
        "dones": [],
    }
    utils_module.trajectories_to_imitations = lambda *args, **kwargs: []

    vairl_module = types.ModuleType("mil_robogym.vairl.vairl")
    vairl_module.VAIRL = DummyVAIRL

    modules = {
        "imitation": imitation_module,
        "imitation.data": data_module,
        "imitation.data.rollout": rollout_module,
        "imitation.util": types.ModuleType("imitation.util"),
        "imitation.util.networks": networks_module,
        "imitation.util.util": util_module,
        "sb3_contrib": sb3_module,
        "mil_robogym.clients.controller_client": controller_client_module,
        "mil_robogym.clients.data_collector_client": data_collector_client_module,
        "mil_robogym.clients.localization_client": localization_client_module,
        "mil_robogym.clients.world_control_client": world_control_client_module,
        "mil_robogym.clients.move_client": move_client_module,
        "mil_robogym.clients.set_pose_client": set_pose_client_module,
        "mil_robogym.vairl.environment": environment_module,
        "mil_robogym.vairl.reward_net": reward_net_module,
        "mil_robogym.vairl.utils": utils_module,
        "mil_robogym.vairl.vairl": vairl_module,
    }
    modules["imitation.util"].networks = networks_module
    modules["imitation.util"].util = util_module

    for module_name, module in modules.items():
        monkeypatch.setitem(sys.modules, module_name, module)


def _load_trainer_module(monkeypatch):
    _install_trainer_import_stubs(monkeypatch)
    sys.modules.pop("mil_robogym.vairl.trainer", None)
    return importlib.import_module("mil_robogym.vairl.trainer")


def _build_trainer(
    module,
    monkeypatch,
    tmp_path: Path,
    *,
    num_episodes: int,
    save_every: int,
):
    trainer = module.Trainer.__new__(module.Trainer)
    trainer.project = {
        "name": "Demo Project",
        "tensor_spec": {"input_features": []},
        "random_spawn_space": {"enabled": False},
    }
    trainer.max_step_count = 1
    trainer.rollout_steps = 32
    trainer.generator_learning_rate = 1e-3
    trainer.discriminator_learning_rate = 3e-3
    trainer.z_size = 6
    trainer.e_hidden_size = 128
    trainer.i_c = 0.5
    trainer.beta_step_size = 1e-3
    trainer.gamma = 0.99
    trainer.save_every = save_every
    trainer.num_episodes = num_episodes
    trainer.seed = 42
    trainer.env_id = "TrainerSaveTest-v0"
    trainer.max_vals = []
    trainer.data_collector_client = types.SimpleNamespace(
        get_logger=lambda: types.SimpleNamespace(info=lambda _message: None),
    )
    trainer.move_client = object()
    trainer.set_pose_client = object()
    trainer.world_control_client = object()
    trainer.controller_client = object()
    trainer.localization_client = object()
    trainer.demo_trajectories = [{"name": "demo_1"}, {"name": "demo_2"}]
    trainer.demo_batches = {}
    trainer.flattened_demo_trajectories = []
    trainer.demos_batch_size = 1
    trainer.eval_environment = object()
    trainer._ready_simulation = lambda: None
    trainer._stop_simulation = lambda: None
    trainer.generate_generator_trajectories = lambda generator, reward_net: (
        [["traj"]],
        1.25,
        0.5,
    )

    source_project_dir = tmp_path / "source_project"
    share_project_dir = tmp_path / "share_project"
    monkeypatch.setattr(
        module,
        "get_training_project_dir_paths",
        lambda _project: [source_project_dir, share_project_dir],
    )

    return trainer, source_project_dir, share_project_dir


def _list_agent_dirs(project_dir: Path) -> list[Path]:
    return sorted((project_dir / "agents").iterdir(), key=lambda path: path.name)


def test_train_saves_periodic_and_final_checkpoints(tmp_path: Path, monkeypatch):
    """Saves both periodic and final generator checkpoints during training."""
    module = _load_trainer_module(monkeypatch)
    trainer, source_project_dir, share_project_dir = _build_trainer(
        module,
        monkeypatch,
        tmp_path,
        num_episodes=3,
        save_every=2,
    )

    trainer.train()

    source_agents = _list_agent_dirs(source_project_dir)
    share_agents = _list_agent_dirs(share_project_dir)

    assert [agent.name for agent in source_agents] == [
        agent.name for agent in share_agents
    ]
    assert len(source_agents) == 2
    assert source_agents[0].name.endswith("_ep_0002")
    assert source_agents[1].name.endswith("_final")

    checkpoint_cfg = yaml.safe_load(
        (source_agents[0] / "config.yaml").read_text(encoding="utf-8"),
    )
    final_cfg = yaml.safe_load(
        (source_agents[1] / "config.yaml").read_text(encoding="utf-8"),
    )
    assert checkpoint_cfg["robogym_agent"]["num_demos"] == 2
    assert checkpoint_cfg["robogym_agent"]["checkpoint_episode"] == 2
    assert final_cfg["robogym_agent"]["num_demos"] == 2
    assert "checkpoint_episode" not in final_cfg["robogym_agent"]

    assert (source_agents[0] / "generator_model.zip").is_file()
    assert (source_agents[1] / "generator_model.zip").is_file()

    checkpoint_rows = (
        (source_agents[0] / "training_metrics.csv")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    )
    final_rows = (
        (source_agents[1] / "training_metrics.csv")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    )
    assert len(checkpoint_rows) == 3
    assert len(final_rows) == 4


def test_train_saves_final_checkpoint_when_periodic_saving_disabled(
    tmp_path: Path,
    monkeypatch,
):
    """Still saves the final generator checkpoint when save_every is disabled."""
    module = _load_trainer_module(monkeypatch)
    trainer, source_project_dir, share_project_dir = _build_trainer(
        module,
        monkeypatch,
        tmp_path,
        num_episodes=2,
        save_every=0,
    )

    trainer.train()

    source_agents = _list_agent_dirs(source_project_dir)
    share_agents = _list_agent_dirs(share_project_dir)

    assert [agent.name for agent in source_agents] == [
        agent.name for agent in share_agents
    ]
    assert len(source_agents) == 1
    assert source_agents[0].name.endswith("_final")
    assert (source_agents[0] / "generator_model.zip").is_file()


def test_resolve_unique_agent_name_skips_existing_agent_folders(
    tmp_path: Path,
    monkeypatch,
):
    """Avoids colliding with existing agent folders during repeated UI runs."""
    module = _load_trainer_module(monkeypatch)
    trainer, source_project_dir, share_project_dir = _build_trainer(
        module,
        monkeypatch,
        tmp_path,
        num_episodes=1,
        save_every=0,
    )

    existing_name = "2026_03_26_03_15_pm_final"
    (source_project_dir / "agents" / existing_name).mkdir(parents=True)
    (share_project_dir / "agents" / existing_name).mkdir(parents=True)

    resolved_name = trainer._resolve_unique_agent_name(
        existing_name,
        [source_project_dir, share_project_dir],
    )

    assert resolved_name == "2026_03_26_03_15_pm_final_run_02"
