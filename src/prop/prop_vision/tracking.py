class Tracker:
    # wrapper for ultralytics bytetrack

    def __init__(self, model, conf=0.25, tracker_cfg="bytetrack.yaml"):
        self.model = model
        self.conf = conf
        self.tracker_cfg = tracker_cfg

    def track(self, frame):

        results = self.model.track(
            frame,
            conf=self.conf,
            tracker=self.tracker_cfg,
            persist=True,
            verbose=False,
        )[0]

        tracks = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            class_id = int(box.cls[0])
            label = self.model.names[class_id]
            track_id = int(box.id[0]) if box.id is not None else None

            tracks.append(
                {
                    "box": (int(x1), int(y1), int(x2), int(y2)),
                    "conf": conf,
                    "class_id": class_id,
                    "label": label,
                    "track_id": track_id,
                },
            )

        return tracks
