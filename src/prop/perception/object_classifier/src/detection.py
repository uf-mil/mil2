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
        annotated = frame.copy()
        h, w = annotated.shape[:2]
        scale = w / 1280
        thick = max(2, round(2 * scale))
        font = cv2.FONT_HERSHEY_SIMPLEX

        for det in detections:
            x1, y1, x2, y2 = det["box"]
            track_id = det.get("track_id")
            label = f'{det["label"]} {det["conf"]:.2f}'
            if track_id is not None:
                label = f"#{track_id} {label}"
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                label,
                (x1, max(y1 - 8, 0)),
                font,
                0.5,
                (0, 255, 0),
                2,
            )

        ranged = sorted(
            (d for d in detections if d.get("range_m") is not None),
            key=lambda d: d["range_m"],
        )
        lines = [f'{d["range_m"]:.1f} m' for d in ranged[:6]] or ["no range"]

        fs = 0.9 * scale
        pad = round(10 * scale)
        sizes = [cv2.getTextSize(t, font, fs, thick)[0] for t in lines]
        line_h = max(s[1] for s in sizes) + pad
        box_w = max(s[0] for s in sizes) + 2 * pad
        box_h = line_h * len(lines) + pad
        x0, y0 = w - box_w - pad, pad

        overlay = annotated.copy()
        cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (201, 0, 57), -1)
        annotated = cv2.addWeighted(overlay, 0.8, annotated, 0.4, 0)

        for i, text in enumerate(lines):
            y = y0 + pad + sizes[i][1] + i * line_h
            cv2.putText(
                annotated,
                text,
                (x0 + pad, y),
                font,
                fs,
                (255, 255, 255),
                thick,
            )

        return annotated
