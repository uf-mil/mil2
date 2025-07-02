import numpy as np
from collections import deque

class SonarTracker:
    def __init__(self):
        # List of (position, direction) tuples
        self.pings = deque(maxlen=10)

    def add_ping(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        """
        Add a ping with a known position and direction vector.
        roll: x
        pitch: y
        yaw: z
        """

        position = np.array([x, y, z])
        direction = np.array([roll, pitch, yaw])

        # ts (this) should already be normalized!! TODO remove this to save time
        direction = direction / np.linalg.norm(direction)
        self.pings.append((position, direction))

    def triangulate(self) -> np.ndarray | None:
        """
        Compute the 3D point that is closest (in least squares sense) to all the rays.
        Returns:
            np.ndarray: Estimated 3D position of the pinger.
        """
        if len(self.pings) < 2:
            print("Need at least two pings to triangulate.")
            return None

        A = np.zeros((3, 3))
        b = np.zeros(3)

        for p, d in self.pings:
            d = d.reshape(3, 1)  # column vector
            I = np.eye(3)
            A_i = I - d @ d.T  # Projection matrix onto plane perpendicular to d
            A += A_i
            b += A_i @ p

        try:
            x = np.linalg.solve(A, b)
            return x
        except np.linalg.LinAlgError:
            print("Degenerate system: can't triangulate")
            return None

def main():
    tracker = SonarTracker()

    # Example pings (positions and directions)
    tracker.add_ping(0.0, 0.0, 0.0, 1.0, 1.0, 0.0)
    tracker.add_ping(1.0, 0.0, 0.0, -1.0, 1.0, 0.0)
    tracker.add_ping(0.5, 1.0, 0.0, 0.0, -1.0, 0.0)

    result = tracker.triangulate()
    if result is not None:
        print("Estimated pinger position:", result)

if __name__ == "__main__":
    main()
