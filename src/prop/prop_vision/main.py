import cv2
from camera import Camera
from detection import Detector
from tracking import Tracker
from undistort import Undistorter


def main():
    cam = Camera(index=1, width=1920, height=1080)
    undistorter = Undistorter("camera_params.json")
    detector = Detector("models/best.pt")
    tracker = Tracker(detector.model)

    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                print("can't receive frame (stream end?).")
                break

            corrected = undistorter.apply(frame)
            tracks = tracker.track(corrected)
            annotated = detector.draw(corrected, tracks)

            cv2.imshow("tracking", annotated)

            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
