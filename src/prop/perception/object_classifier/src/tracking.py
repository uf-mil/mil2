import numpy as np
from ultralytics.engine.results import Boxes
from ultralytics.trackers import BOTSORT, BYTETracker
from ultralytics.utils import YAML, IterableSimpleNamespace
from ultralytics.utils.checks import check_yaml
from utils import suppress_nested

TRACKERS = {"botsort": BOTSORT, "bytetrack": BYTETracker}


class Tracker:
    # wrapper for ultralytics botsort
    def __init__(self, model, conf=0.25, iou=0.5, tracker_cfg="botsort.yaml"):
        self.model = model
        self.conf = conf
        self.iou = iou
        cfg = IterableSimpleNamespace(**YAML.load(check_yaml(tracker_cfg)))
        self.tracker = TRACKERS[cfg.tracker_type](args=cfg)

    def _detect(self, frame):
        result = self.model.predict(frame, conf=self.conf, iou=self.iou, verbose=False)[
            0
        ]
        dets = []
        for box in result.boxes:
            dets.append(
                {
                    "box": tuple(box.xyxy[0].tolist()),
                    "conf": float(box.conf[0]),
                    "class_id": int(box.cls[0]),
                },
            )
        return dets

    def track(self, frame):
        dets = suppress_nested(self._detect(frame))
        rows = np.array(
            [[*d["box"], d["conf"], d["class_id"]] for d in dets],
            dtype=np.float32,
        ).reshape(-1, 6)
        out = self.tracker.update(Boxes(rows, frame.shape[:2]), frame)

        tracks = []
        for x1, y1, x2, y2, track_id, score, cls, _ in out:
            class_id = int(cls)
            tracks.append(
                {
                    "box": (int(x1), int(y1), int(x2), int(y2)),
                    "conf": float(score),
                    "class_id": class_id,
                    "label": self.model.names[class_id],
                    "track_id": int(track_id),
                },
            )
        return tracks

    def reset(self):
        self.tracker.reset()
