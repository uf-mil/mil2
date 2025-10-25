from abc import ABC, abstractmethod

from cv2.typing import MatLike

"""
If you want to track some object, you need a class that implements this class
"""


class CentroidFinder(ABC):
    @property
    @abstractmethod
    def topic_name(self) -> str:
        pass

    # returns (x, y) pair of centroid or None if not found
    @abstractmethod
    def find_centroid(self, frame: MatLike) -> tuple[int, int] | None:
        pass
