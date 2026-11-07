"""
Carries the cameras' classes on the evidence grid, so they share its memory.

The grid already decides what is there: cells build up while the lidar hits them
and fade only when the lidar should have seen them and did not. Here its solid
cells are grouped into clusters, a labelled box names the nearby cells of the
cluster it sits on, and a label lasts until its cell has no evidence left. A buoy
that is hidden, or out of every camera's view, keeps its class.

No ROS here, so it tests on its own.
"""

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

_ALL_NEIGHBOURS = np.ones((3, 3), dtype=bool)


def clusters_of(cells):
    """Number each touching group of True cells from 1, leaving the rest 0."""
    clusters, _ = ndimage.label(cells, structure=_ALL_NEIGHBOURS)
    return clusters


@dataclass(frozen=True)
class Window:
    """The part of the world a grid covers: its corner, and metres per cell."""

    x: float
    y: float
    resolution: float

    @property
    def offset(self):
        """
        How many rows and columns the corner sits from the world's origin.

        The window only ever moves in whole cells, so rounding recovers the count.
        """
        return round(self.y / self.resolution), round(self.x / self.resolution)

    def to_world(self, row, column):
        """A cell's indices counted from the world's origin, fixed as the window moves."""
        rows, columns = self.offset
        return row + rows, column + columns

    def from_world(self, cell):
        """The inverse of to_world, for the window as it is now."""
        rows, columns = self.offset
        return cell[0] - rows, cell[1] - columns

    def centres(self, rows, columns):
        """The x and y, in metres, of the middle of each cell."""
        return (
            self.x + (columns + 0.5) * self.resolution,
            self.y + (rows + 0.5) * self.resolution,
        )


@dataclass(frozen=True)
class LabelledObject:
    """One labelled thing on the grid: its class, and its centre and size in metres."""

    label: str
    x: float
    y: float
    size_x: float
    size_y: float


class ObjectMap:
    """
    The labelled objects on an evidence grid.

    occupied is the evidence that makes a cell solid. label_radius is how far
    from a labelled position its class reaches, in metres.

    max_cluster_size is the widest cluster that can take a label, in metres. A
    label names something free-standing: anything wider is a dock or a shoreline,
    even with a buoy touching it, because the lidar cannot tell the two apart.
    """

    def __init__(self, occupied, label_radius, max_cluster_size):
        self.occupied = occupied
        self.label_radius = label_radius
        self.max_cluster_size = max_cluster_size
        self._evidence = np.zeros((0, 0), dtype=np.int8)
        self._clusters = np.zeros((0, 0), dtype=np.int32)
        self._window = Window(0.0, 0.0, 1.0)
        self._labels = {}  # world (row, column) -> class name

    def update_grid(self, evidence, window):
        """Take a new grid: forget labels on empty cells, then extend the others."""
        self._evidence = evidence
        self._window = window
        self._clusters = clusters_of(evidence >= self.occupied)
        self._labels = {
            cell: label
            for cell, label in self._labels.items()
            if self._has_evidence(cell)
        }
        for found in self.objects():
            for cell in self._cluster_cells_near(found.x, found.y):
                self._labels.setdefault(cell, found.label)

    def add_label(self, label, x, y):
        """Name the cluster at (x, y). A newer label replaces an older one."""
        for cell in self._cluster_cells_near(x, y):
            self._labels[cell] = label

    def objects(self):
        """Each touching group of cells sharing a class, as one LabelledObject."""
        found = []
        for label in sorted(set(self._labels.values())):
            groups = clusters_of(self._cells_labelled(label))
            for group in range(1, groups.max() + 1):
                found.append(self._describe(label, groups == group))
        return found

    def _has_evidence(self, cell):
        row, column = self._window.from_world(cell)
        height, width = self._evidence.shape
        return (
            0 <= row < height
            and 0 <= column < width
            and self._evidence[row, column] > 0
        )

    def _cluster_cells_near(self, x, y):
        """World cells of the closest cluster, within label_radius of (x, y)."""
        rows, columns = np.nonzero(self._clusters)
        cell_x, cell_y = self._window.centres(rows, columns)
        distances = np.hypot(cell_x - x, cell_y - y)
        near = distances <= self.label_radius
        if not near.any():
            return []

        clusters = self._clusters[rows, columns]
        in_cluster = clusters == clusters[np.argmin(distances)]
        if self._too_wide(rows[in_cluster], columns[in_cluster]):
            return []

        chosen = near & in_cluster
        return [
            self._window.to_world(row, column)
            for row, column in zip(rows[chosen].tolist(), columns[chosen].tolist())
        ]

    def _too_wide(self, rows, columns):
        cells_across = max(np.ptp(rows), np.ptp(columns)) + 1
        return cells_across * self._window.resolution > self.max_cluster_size

    def _cells_labelled(self, label):
        cells = np.zeros(self._evidence.shape, dtype=bool)
        for cell, name in self._labels.items():
            if name == label:
                cells[self._window.from_world(cell)] = True
        return cells

    def _describe(self, label, cells):
        rows, columns = np.nonzero(cells)
        x, y = self._window.centres(rows, columns)
        resolution = self._window.resolution
        return LabelledObject(
            label=label,
            x=float(x.mean()),
            y=float(y.mean()),
            size_x=float(x.max() - x.min()) + resolution,
            size_y=float(y.max() - y.min()) + resolution,
        )
