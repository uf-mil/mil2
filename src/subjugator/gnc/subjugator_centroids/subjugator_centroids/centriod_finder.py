from abc import ABC, abstractmethod
from cv2.typing import MatLike

"""
If you want to track some object, you need a class that implements this class
"""
class CentriodFinder(ABC):
    # should be self explanitory
    @property
    @abstractmethod
    def topic_name(self) -> str:
        pass

    # returns (x, y) pair of centriod or None if not found
    @abstractmethod
    def find_centriod(self, frame: MatLike) -> tuple[int, int] | None:
        pass

