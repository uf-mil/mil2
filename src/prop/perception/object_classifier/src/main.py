import sys

import cv2
from camera import Camera, parse_source
from prop.perception.object_classifier.src.detection import Detector
from prop.perception.object_classifier.src.localization import Localizer
from tracking import Tracker
from undistort import Undistorter
from utils import load_config


def print_estimates(tracks):
    if not tracks:
        print("(no detections)")
    for t in tracks:
        _, y1, _, y2 = t["box"]
        head = f'#{t.get("track_id")} {t["label"]}: box height {y2 - y1}px, bearing {t["bearing_deg"]:+.1f} deg'
        if t["range_m"] is None:
            print(f'{head}, no range ({t["range_note"]})')
        else:
            print(f'{head}, Z={t["z_m"]:.2f} m, range={t["range_m"]:.2f} m')


def main():
    source = parse_source(sys.argv[1]) if len(sys.argv) > 1 else load_config()["camera_index"]
    cam = Camera(source, width=1920, height=1080)
    undistorter = Undistorter()
    detector = Detector("models/best.pt")
    tracker = Tracker(detector.model)
    localizer = Localizer()

    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                print("can't receive frame (stream end?).")
                break

            corrected = undistorter.apply(frame)
            tracks = tracker.track(corrected)
            for t in tracks:
                t.update(localizer.locate(t, undistorter.intrinsics))

            annotated = detector.draw(corrected, tracks)
            cv2.imshow("tracking", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("p"):
                print_estimates(tracks)
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
