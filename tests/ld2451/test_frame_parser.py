import pytest

from ld2451.serial_reader import Frame
from ld2451.frame_parser import parse


def make_frame(payload: bytes) -> Frame:
    """Helper to construct a Frame object for testing."""
    return Frame(
        header=b"\xF4\xF3\xF2\xF1",
        length=len(payload),
        payload=payload,
        trailer=b"\xF8\xF7\xF6\xF5",
    )


def test_parse_empty_frame():
    """
    No targets present.
    """

    payload = bytes([
        0,      # target count
        0       # approaching detected
    ])

    radar = parse(make_frame(payload))

    assert radar.target_count == 0
    assert radar.approaching_detected is False
    assert radar.targets == []


def test_parse_single_target():
    """
    Parse one approaching target.
    """

    payload = bytes([
        1,          # target count
        1,          # approaching detected

        0x8A,       # angle = 10°
        42,         # distance = 42 m
        0,          # approaching
        55,         # speed = 55 km/h
        200         # snr
    ])

    radar = parse(make_frame(payload))

    assert radar.target_count == 1
    assert radar.approaching_detected is True

    target = radar.targets[0]

    assert target.angle == 10
    assert target.distance == 42
    assert target.approaching is True
    assert target.speed == 55
    assert target.snr == 200


def test_parse_two_targets():

    payload = bytes([
        2,
        1,

        0x85,
        20,
        0,
        40,
        100,

        0x78,
        80,
        1,
        25,
        180
    ])

    radar = parse(make_frame(payload))

    assert len(radar.targets) == 2

    assert radar.targets[0].angle == 5
    assert radar.targets[0].approaching is True

    assert radar.targets[1].angle == -8
    assert radar.targets[1].approaching is False


def test_payload_too_short():

    payload = bytes([1])

    with pytest.raises(ValueError):
        parse(make_frame(payload))


def test_incomplete_target_data():

    payload = bytes([
        1,
        0,

        0x81,
        15
    ])

    with pytest.raises(ValueError):
        parse(make_frame(payload))

def test_angle_conversion():

    payload = bytes([
        3,
        0,

        0x80, 10, 0, 10, 10,
        0x81, 10, 0, 10, 10,
        0x7F, 10, 0, 10, 10,
    ])

    radar = parse(make_frame(payload))

    assert radar.targets[0].angle == 0
    assert radar.targets[1].angle == 1
    assert radar.targets[2].angle == -1