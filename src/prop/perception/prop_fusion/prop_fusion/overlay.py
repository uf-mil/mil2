"""Draws what fusion saw onto a camera image."""

import cv2

DETECTION = (0, 255, 0)
TRACK = (0, 220, 255)
PAIR = (255, 0, 255)


def draw(frame, detections, tracks, pairs):
    """
    detections, tracks: lists of ((left, top, right, bottom), text).
    pairs: (track index, detection index) tuples.
    """
    for box, text in detections:
        _box(frame, box, text, DETECTION, below=True)
    for box, text in tracks:
        _box(frame, box, text, TRACK, below=False)
    for i, j in pairs:
        cv2.line(frame, _centre(tracks[i][0]), _centre(detections[j][0]), PAIR, 2)
    return frame


def _box(frame, box, text, colour, below):
    left, top, right, bottom = box
    cv2.rectangle(frame, _pt(left, top), _pt(right, bottom), colour, 2)
    anchor = _pt(left, bottom + 18) if below else _pt(left, top - 6)
    cv2.putText(frame, text, anchor, cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)


def _centre(box):
    left, top, right, bottom = box
    return _pt((left + right) / 2, (top + bottom) / 2)


def _pt(u, v):
    return int(round(u)), int(round(v))
