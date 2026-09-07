"""Probe summarisers.

The probe *mechanism* is generic; only its configuration is per-task. A
detection stream is a detection stream whether it carries table markers for
Task 5 or gate poles for Task 1, so these functions know nothing about either.

  detection samples : list of (sim_s, [(label, confidence), ...])
  model samples     : dict model_name -> list of (sim_s, x, y, z)
"""

from __future__ import annotations

import math


def summarize_detections(frames: list) -> dict:
    """Rate, dropouts, and per-class visibility for one detection topic.

    Raw counts scale with run length, so presence (the fraction of frames a
    class appeared in) is reported alongside them -- that is the number that
    says whether an object was reliably visible. min_conf is kept because a
    class with a good mean and a low floor is one that flickers at the
    threshold, which downstream shows up as a target the tree keeps losing.
    """
    if not frames:
        return {"frames": 0, "rate_hz": None, "longest_dropout_s": None, "classes": {}}

    span = frames[-1][0] - frames[0][0]
    rate = len(frames) / span if span > 0 else None

    longest = 0.0
    for (t0, _), (t1, _) in zip(frames, frames[1:]):
        longest = max(longest, t1 - t0)

    classes: dict = {}
    for _, detections in frames:
        seen_this_frame = set()
        for label, conf in detections:
            entry = classes.setdefault(
                label,
                {"detections": 0, "present_frames": 0, "confs": []},
            )
            entry["detections"] += 1
            entry["confs"].append(conf)
            if label not in seen_this_frame:
                entry["present_frames"] += 1
                seen_this_frame.add(label)

    for entry in classes.values():
        confs = entry.pop("confs")
        entry["mean_conf"] = sum(confs) / len(confs)
        entry["min_conf"] = min(confs)
        entry["presence"] = entry["present_frames"] / len(frames)

    return {
        "frames": len(frames),
        "rate_hz": rate,
        "longest_dropout_s": longest,
        "classes": classes,
    }


def summarize_model_poses(models: dict) -> dict:
    """Start, end, displacement, and lift height for each tracked gz model."""
    summary: dict = {}
    for name, samples in models.items():
        if not samples:
            summary[name] = {"present": False}
            continue
        _, x0, y0, z0 = samples[0]
        _, x1, y1, z1 = samples[-1]
        summary[name] = {
            "present": True,
            "start": (round(x0, 3), round(y0, 3), round(z0, 3)),
            "end": (round(x1, 3), round(y1, 3), round(z1, 3)),
            "displacement": math.dist((x0, y0, z0), (x1, y1, z1)),
            "max_displacement": max(
                math.dist((x0, y0, z0), (x, y, z)) for _, x, y, z in samples
            ),
            "max_z_rise": max(z - z0 for _, _x, _y, z in samples),
        }
    return summary
