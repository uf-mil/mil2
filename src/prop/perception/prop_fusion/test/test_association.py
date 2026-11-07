from prop_fusion.association import Votes, associate


def test_associate_is_one_to_one_and_nearest():
    tracks = [(100.0, 120.0), (300.0, 320.0)]
    detections = [(302.0, 318.0), (98.0, 124.0)]
    assert sorted(associate(tracks, detections, 10.0)) == [(0, 1), (1, 0)]


def test_far_detection_is_left_unmatched():
    assert associate([(100.0, 120.0)], [(200.0, 220.0)], 10.0) == []


def test_detection_just_outside_the_span_matches_within_tolerance():
    assert associate([(100.0, 120.0)], [(125.0, 140.0)], 10.0) == [(0, 0)]


def test_one_detection_goes_to_one_track():
    assert associate([(100.0, 120.0), (105.0, 125.0)], [(104.0, 124.0)], 10.0) == [
        (1, 0),
    ]


def test_votes_need_a_clear_lead():
    votes = Votes(margin=2.0)
    for _ in range(3):
        votes.add(7, "red_buoy", 0.8)
    assert votes.label(7) == "red_buoy"
    for _ in range(2):
        votes.add(7, "green_buoy", 0.8)
    assert votes.label(7) == ""
    assert votes.label(8) == ""


def test_votes_forget_dropped_tracks():
    votes = Votes(margin=0.5)
    votes.add(1, "red_buoy", 1.0)
    votes.add(2, "green_buoy", 1.0)
    votes.keep({2})
    assert votes.label(1) == ""
    assert votes.label(2) == "green_buoy"
