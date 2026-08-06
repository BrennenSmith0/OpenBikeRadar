"""
frame_parser.py

Parser for LD2451 intra-frame data reports.

This module converts validated Frame objects from serial_reader.py
into Python objects representing the radar targets.

Protocol:

Byte 0   Target Quantity
Byte 1   Alarm Information

Then N targets, each 5 bytes:

Byte 0   Angle (actual = value - 0x80)
Byte 1   Distance (meters)
Byte 2   Speed Direction
            0 = Approaching
            1 = Moving Away
Byte 3   Speed (km/h)
Byte 4   SNR
"""

from __future__ import annotations
from dataclasses import dataclass
import math
from .serial_reader import Frame
from enum import Enum

class Direction(Enum):
    APPROACHING = 0
    MOVING_AWAY = 1

@dataclass(frozen=True)
class Target:
    """
    One detected radar target.
    """

    angle: int
    distance: int
    direction: Direction
    speed: int
    snr: int

    @property 
    def angle_radians(self) -> float:
        return math.radians(self.angle)
    
    @property
    def closing_speed_mps(self) -> float:
        return self.speed / 3.6

@dataclass(frozen=True)
class RadarFrame:
    """
    Parsed radar frame.
    """

    target_count: int
    approaching_detected: bool
    targets: list[Target]


def parse(frame: Frame) -> RadarFrame:
    """
    Parse an LD2451 radar frame.

    Parameters
    ----------
    frame : Frame
        A validated Frame returned by SerialReader.

    Returns
    -------
    RadarFrame
    """

    payload = frame.payload

    if len(payload) < 2:
        raise ValueError("Payload too short.")

    target_count = payload[0]
    approaching_detected = payload[1] == 1

    expected_length = 2 + target_count * 5

    if len(payload) < expected_length:
        raise ValueError(
            f"Incomplete payload. "
            f"Expected {expected_length} bytes, "
            f"received {len(payload)}."
        )

    targets: list[Target] = []

    offset = 2

    for _ in range(target_count):

        raw_angle = payload[offset]
        distance = payload[offset + 1]
        direction = Direction(payload[offset + 2])
        speed = payload[offset + 3]
        snr = payload[offset + 4]

        target = Target(
            angle=raw_angle - 0x80,
            distance=distance,
            approaching=(direction == 0),
            speed=speed,
            snr=snr,
        )

        targets.append(target)

        offset += 5

    return RadarFrame(
        target_count=target_count,
        approaching_detected=approaching_detected,
        targets=targets,
    )