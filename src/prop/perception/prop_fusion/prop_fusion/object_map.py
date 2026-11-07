"""
Carries the cameras' classes on the evidence grid, so they share its memory.

The grid already decides what is there: cells build up while the lidar hits them
and fade only when the lidar should have seen them and did not. Here its solid
cells are grouped into clusters, and a labelled box makes the nearby cells of the
cluster it sits on part of an object, until they have no evidence left. A buoy
that is hidden, or out of every camera's view, keeps its class and its id.

An object's id comes from where it is, not from the tracker: a buoy can go through
many tracks, and a track can move to something else. A box near an object's cells
is that object, so the id lasts for as long as the grid remembers those cells.

No ROS here, so it tests on its own.
"""

import math
from dataclasses import dataclass
from itertools import count

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
    """One labelled thing on the grid: its id, class, and centre and size in metres."""

    id: int
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
        self._object_at = {}  # world (row, column) -> object id
        self._class_of = {}  # object id -> class name
        self._new_ids = count(1)

    def update_grid(self, evidence, window):
        """Take a new grid: let empty cells go, then extend each object."""
        self._evidence = evidence
        self._window = window
        self._clusters = clusters_of(evidence >= self.occupied)
        self._object_at = {
            cell: object_id
            for cell, object_id in self._object_at.items()
            if self._has_evidence(cell)
        }
        for found in self.objects():
            for cell in self._cluster_cells_near(found.x, found.y):
                self._object_at.setdefault(cell, found.id)

    def add_label(self, label, x, y):
        """
        Name the cluster at (x, y), as the object already there or else a new one.

        A newer label replaces an older one, and the object keeps its id.
        """
        cells = self._cluster_cells_near(x, y)
        if not cells:
            return
        object_id = self._object_near(x, y)
        if object_id is None:
            object_id = next(self._new_ids)
        self._class_of[object_id] = label
        for cell in cells:
            self._object_at[cell] = object_id

    def objects(self):
        """Every object on the grid, oldest first."""
        cells_of = {}
        for cell, object_id in self._object_at.items():
            cells_of.setdefault(object_id, []).append(cell)
        return [self._describe(i, cells_of[i]) for i in sorted(cells_of)]

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

    def _object_near(self, x, y):
        """The object with the closest cell within label_radius of (x, y), if any."""
        closest, nearest = None, self.label_radius
        for cell, object_id in self._object_at.items():
            cell_x, cell_y = self._window.centres(*self._window.from_world(cell))
            distance = math.hypot(cell_x - x, cell_y - y)
            if distance <= nearest:
                closest, nearest = object_id, distance
        return closest

    def _too_wide(self, rows, columns):
        cells_across = max(np.ptp(rows), np.ptp(columns)) + 1
        return cells_across * self._window.resolution > self.max_cluster_size

    def _largest_piece(self, cells):
        """
        The rows and columns of the biggest touching group among an object's cells.

        odom can swing a buoy a metre or two and back, leaving cells in both
        places. An average of the two would sit on open water.
        """
        rows, columns = np.array([self._window.from_world(c) for c in cells]).T
        marked = np.zeros(self._evidence.shape, dtype=bool)
        marked[rows, columns] = True
        pieces = clusters_of(marked)[rows, columns]
        largest = pieces == np.argmax(np.bincount(pieces))
        return rows[largest], columns[largest]

    def _describe(self, object_id, cells):
        rows, columns = self._largest_piece(cells)
        x, y = self._window.centres(rows, columns)
        resolution = self._window.resolution
        return LabelledObject(
            id=object_id,
            label=self._class_of[object_id],
            x=float(x.mean()),
            y=float(y.mean()),
            size_x=float(x.max() - x.min()) + resolution,
            size_y=float(y.max() - y.min()) + resolution,
        )
