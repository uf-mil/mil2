from __future__ import annotations

from typing import Literal, NotRequired, TypedDict

Coord4D = tuple[float, float, float, float]
Coord4DList = list[float]
StateActionPair = tuple[
    dict[str, int | float | complex],  # State
    dict[str, int | float | complex],  # Action
]


class RandomSpawnSpace(TypedDict):
    enabled: bool
    coord1_4d: Coord4D
    coord2_4d: Coord4D


class RoboGymTensorSpec(TypedDict):
    input_features: list[str]
    output_features: list[str]
    input_dim: int
    output_dim: int


SupportedNonNumericDataType = Literal["unordered_set", "image"]


class NonNumericTopicFieldSelection(TypedDict):
    field_path: str
    data_type: SupportedNonNumericDataType
    ros_type: str


class RandomSpawnSpaceConfig(TypedDict):
    enabled: bool
    coord1_4d: Coord4DList
    coord2_4d: Coord4DList


class RoboGymProjectYaml(TypedDict):
    name: str
    world_file: str
    model_name: str
    random_spawn_space: RandomSpawnSpaceConfig
    input_topics: dict[str, list[str]]
    output_topics: dict[str, list[str]]
    input_non_numeric_topics: NotRequired[
        dict[str, list[NonNumericTopicFieldSelection]]
    ]
    output_non_numeric_topics: NotRequired[
        dict[str, list[NonNumericTopicFieldSelection]]
    ]
    tensor_spec: NotRequired[RoboGymTensorSpec]


class RoboGymTrainingYaml(TypedDict):
    num_episodes: int
    rollout_steps: int
    generator_learning_rate: float
    discriminator_learning_rate: float
    z_size: int
    e_hidden_size: int
    i_c: float
    beta_step_size: float
    gamma: float
    save_every: int
    seed: int
    max_step_count: int | None
    expert_noise_std: float


class RoboGymProjectConfig(TypedDict):
    robogym_project: RoboGymProjectYaml
    robogym_training: NotRequired[RoboGymTrainingYaml]


class RoboGymAgentYaml(TypedDict):
    name: str
    num_demos: int
    model_file: str
    checkpoint_episode: NotRequired[int]
    training_settings: NotRequired[RoboGymTrainingYaml]


class RoboGymAgentConfig(TypedDict):
    robogym_agent: RoboGymAgentYaml


class RoboGymDemoYaml(TypedDict):
    name: str
    start_position: Coord4DList
    sampling_rate: float


class RoboGymDemoConfig(TypedDict):
    robogym_demo: RoboGymDemoYaml


FlattenedTopic = dict[str, object]
SampledTopics = dict[str, FlattenedTopic]


class TopicWarning(TypedDict):
    topic: str
    reason: str
    matched: list[str]
    category: str
