import cv2


class Camera:
    # set initial camera settings for unwarp.
    def __init__(self, index=0, width=1920, height=1080, fsp=30, backend=cv2.CAP_MSMF):
        self.width = width
        self.height = height

        self.cap = cv2.VideoCapture(index, backend)
        if not self.cap.isOpened():
            raise RuntimeError(f"unable to open camera at index {index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
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
