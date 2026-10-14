import sys

import cv2


def default_backend():
    if sys.platform.startswith("win"):
        return cv2.CAP_MSMF
    if sys.platform.startswith("linux"):
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


class Camera:
    # set initial camera settings for unwarp.
    def __init__(
        self,
        index=0,
        width=1920,
        height=1080,
        fps=30,
        backend=default_backend(),
    ):
        self.width = width
        self.height = height

        self.cap = cv2.VideoCapture(index, backend)
        if not self.cap.isOpened():
            raise RuntimeError(f"unable to open camera at index {index}")

        mjpg = cv2.VideoWriter_fourcc(*"MJPG")
        linux = sys.platform.startswith("linux")
        if linux:
            self.cap.set(cv2.CAP_PROP_FOURCC, mjpg)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not linux:
            self.cap.set(cv2.CAP_PROP_FOURCC, mjpg)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self._check_set()

    def _check_set(self):
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        print(f"camera settings set: {fourcc_str} @ {actual_w}x{actual_h}")
        if (actual_w, actual_h) != (self.width, self.height):
            print(
                f"requested {self.width}x{self.height}, but "
                f"got {actual_w}x{actual_h}",
            )

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()
