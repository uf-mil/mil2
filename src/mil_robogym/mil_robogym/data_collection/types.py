from __future__ import annotations

from typing import NotRequired, TypedDict

Coord4D = tuple[float, float, float, float]
Coord4DList = list[float]


class RandomSpawnSpace(TypedDict):
    enabled: bool
    coord1_4d: Coord4D
    coord2_4d: Coord4D


class RoboGymTensorSpec(TypedDict):
    input_features: list[str]
    output_features: list[str]
    input_dim: int
    output_dim: int
    ignored_input_features: dict[str, list[str]]
    ignored_output_features: dict[str, list[str]]


class RandomSpawnSpaceConfig(TypedDict):
    enabled: bool
    coord1_4d: Coord4DList
    coord2_4d: Coord4DList


class RoboGymProjectYaml(TypedDict):
    name: str
    world_file: str
    model_name: str
    random_spawn_space: RandomSpawnSpaceConfig
    input_topics: list[str]
    output_topics: list[str]
    tensor_spec: NotRequired[RoboGymTensorSpec]


class RoboGymProjectConfig(TypedDict):
    robogym_project: RoboGymProjectYaml


class RoboGymDemoYaml(TypedDict):
    demo_name: str
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


class RoboGymProject(TypedDict):
    project_name: str
    world_file: str
    model_name: str
    random_spawn_space: RandomSpawnSpace
    input_topics: list[str]
    output_topics: list[str]
    tensor_spec: NotRequired[RoboGymTensorSpec]


class RoboGymDemoConfig(TypedDict):
    robogym_demo: RoboGymDemo


class RoboGymDemo(TypedDict):
    demo_name: str
    start_position: Coord4D
    sampling_rate: float
