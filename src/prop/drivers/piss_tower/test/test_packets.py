from piss_tower.piss_tower import make_msg, packets


def test_set_manual_packet_matches_the_board_protocol():
    assert make_msg(0x10, 0x02, b"\x00").hex(" ") == "37 01 10 02 01 00 00 13 5b"


def test_packets_round_trip_through_noise_and_splits():
    stream = b"\xff\x37" + make_msg(0x10, 0x02, b"\x01") + make_msg(0x20, 0x01, b"")
    found, rest = packets(stream[:7])
    assert found == []
    found, rest = packets(rest + stream[7:])
    assert found == [(0x10, 0x02, b"\x01", True), (0x20, 0x01, b"", True)]
    assert rest == b""


def test_bad_checksum_is_flagged():
    packet = bytearray(make_msg(0x10, 0x02, b"\x01"))
    packet[-1] ^= 0xFF
    found, _ = packets(bytes(packet))
    assert found == [(0x10, 0x02, b"\x01", False)]
