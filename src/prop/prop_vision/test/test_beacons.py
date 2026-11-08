import numpy as np
import pytest
from prop_vision.beacons import OFF, Beacons, lit_color, pattern

BGR = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 0, 0)}


def crop_with_light(color=None, pixels=12, size=(40, 30)):
    """A grey crop with a square LED of about `pixels` pixels."""
    crop = np.full((size[1], size[0], 3), 90, np.uint8)
    if color:
        side = max(1, int(round(pixels**0.5)))
        crop[10 : 10 + side, 12 : 12 + side] = BGR[color]
    return crop


def timeline(color, kind, seconds=4.0, fps=30, phase=0.0, dropout=0.0, seed=0):
    """(t, colour) as the camera would see a light that is flashing, steady or off."""
    rng = np.random.default_rng(seed)
    history = []
    for i in range(int(seconds * fps)):
        t = i / fps
        on = kind == "steady" or (kind == "flashing" and (t + phase) % 2.0 < 1.0)
        seen = color if on and rng.random() >= dropout else None
        history.append((t, seen))
    return history


@pytest.mark.parametrize("color", ["red", "green", "blue"])
def test_a_small_led_is_its_colour(color):
    assert lit_color(crop_with_light(color, pixels=9)) == color


def test_dark_crops_and_specks_are_not_lit():
    assert lit_color(crop_with_light(None)) is None
    assert lit_color(crop_with_light("red", pixels=1)) is None


def test_a_colour_filling_the_crop_is_not_an_led():
    crop = np.zeros((20, 20, 3), np.uint8)
    crop[:] = BGR["red"]
    assert lit_color(crop) is None


@pytest.mark.parametrize("fps", [10, 30])
@pytest.mark.parametrize("phase", [0.0, 0.4, 1.0, 1.7])
@pytest.mark.parametrize(
    "color,state",
    [("red", "FLASHING_RED"), ("green", "FLASHING_GREEN"), ("blue", "FLASHING_BLUE")],
)
def test_flashing_lights(color, state, phase, fps):
    assert pattern(timeline(color, "flashing", fps=fps, phase=phase)) == state


def test_flashing_survives_dropped_frames():
    history = timeline("green", "flashing", dropout=0.15, seed=3)
    assert pattern(history) == "FLASHING_GREEN"


def test_steady_blue_and_off():
    assert pattern(timeline("blue", "steady")) == "STEADY_BLUE"
    assert pattern(timeline(None, "off")) == OFF


def test_steady_red_is_not_a_task_state():
    assert pattern(timeline("red", "steady")) is None


def test_too_short_to_say():
    assert pattern(timeline("red", "flashing", seconds=2.0)) is None


def feed(beacons, object_id, history, start=0.0, close=True):
    for t, color in history:
        beacons.add(object_id, color, start + t, close)


def test_votes_decide_and_stay_with_the_object():
    beacons = Beacons(min_votes=15)
    feed(beacons, 7, timeline("red", "flashing", seconds=6.0))
    assert beacons.state(7) == "FLASHING_RED"
    assert beacons.state(8) is None


def test_off_only_counts_when_close():
    beacons = Beacons(min_votes=15)
    feed(beacons, 3, timeline(None, "off", seconds=6.0), close=False)
    assert beacons.state(3) is None
    feed(beacons, 3, timeline(None, "off", seconds=6.0), start=100.0, close=True)
    assert beacons.state(3) == OFF


def test_a_gap_starts_the_history_again():
    beacons = Beacons(min_votes=1)
    feed(beacons, 1, timeline("blue", "steady", seconds=2.0))
    feed(beacons, 1, timeline("blue", "steady", seconds=2.0), start=10.0)
    assert beacons.state(1) is None


def test_forget_drops_ended_objects():
    beacons = Beacons(min_votes=1)
    feed(beacons, 1, timeline("blue", "steady"))
    feed(beacons, 2, timeline("blue", "steady"))
    beacons.forget({2})
    assert beacons.state(1) is None
    assert beacons.state(2) == "STEADY_BLUE"
