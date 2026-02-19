from datetime import datetime
from typing import TypedDict


class Pose(TypedDict):
    """
    Tranlational position and orientation.
    """

    model_name: str
    last_update: datetime

    x: float
    y: float
    z: float

    roll: float
    pitch: float
    yaw: float
