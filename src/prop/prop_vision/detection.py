import cv2
from ultralytics import YOLO


class Detector:
    # run YOLO!

    def __init__(self, model_path="models/best.pt", conf=0.25):
        self.model = YOLO(model_path)
        self.conf = conf

    def detect(self, frame):
        results = self.model(frame, conf=self.conf, verbose=False)[0]

        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            class_id = int(box.cls[0])
            label = self.model.names[class_id]

            detections.append(
                {
                    "box": (int(x1), int(y1), int(x2), int(y2)),
                    "conf": conf,
                    "class_id": class_id,
                    "label": label,
                },
            )

        return detections

    @staticmethod
    def draw(frame, detections):
        # draw bounding boxes and labels on a copy of the frame, then returns the annotated copy
        annotated = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = det["box"]
            track_id = det.get("track_id")
            label = (
                f'#{track_id} {det["label"]} {det["conf"]:.2f}'
                if track_id is not None
                else f'{det["label"]} {det["conf"]:.2f}'
            )
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                label,
                (x1, max(y1 - 8, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
        return annotated
