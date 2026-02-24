from __future__ import annotations

from typing import TypedDict

Coord4D = tuple[float, float, float, float]


class RandomSpawnSpace(TypedDict):
    enabled: bool
    coord1_4d: Coord4D
    coord2_4d: Coord4D


class RoboGymProject(TypedDict):
    project_name: str
    world_file: str
    model_name: str
    random_spawn_space: RandomSpawnSpace
    input_topics: list[str]
    output_topics: list[str]
