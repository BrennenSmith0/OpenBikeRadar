"""
frame_parser.py

Parser for LD2451 intra-frame data reports.

This module converts validated Frame objects from serial_reader.py
into Python objects representing the radar targets.

Protocol for the body:

Byte 0   Target Quantity
Byte 1   Alarm Information, 0 if no target is approaching, 1 if a target is approaching

Then N targets, each 5 bytes:

Byte 0   Angle (actual = value - 0x80)
Byte 1   Distance (meters) 0 - 100
Byte 2   Speed Direction, this is flipped from the documentation for my sensor
            0 = Moving Away
            1 = Approaching
Byte 3   Speed (km/h)
Byte 4   SNR
"""

from __future__ import annotations
from dataclasses import dataclass
import math
from .serial_reader import Frame
from .enums import Direction


@dataclass(frozen=True)
class RadarTarget:
    """
    One detected radar target.
    """

    angle: int
    distance: int
    direction: Direction
    speed: int
    snr: int

    def __str__(self):

        if self.direction == Direction.APPROACHING:
            direction = "Approaching"
        else:
            direction = "Moving Away"
        
        return (
            f"{direction} "
            f"{self.distance:3}m "
            f"{self.angle:+3}° "
            f"{self.speed:2}km/h "
            f"SNR={self.snr}"
        )

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
    targets: list[RadarTarget]


def parse(frame: Frame) -> RadarFrame:
    """
    Parse an LD2451 radar frame.

    Parameters
    ----------
    frame : Frame
        A validated Frame returned by SerialReader from serial_reader.py

    Returns
    -------
    RadarFrame
    """

    payload = frame.payload
    # Handle the empty "no target" frames this sensor sends
    if len(payload) == 0:
        return RadarFrame(
            target_count=0,
            approaching_detected=False,
            targets=[],
        )
  

    if len(payload) < 2:
        raise ValueError("Payload too short.")

    #byte 0 is number of targets which each being 5 bytes with a trailer of 2 bytes
    target_count = payload[0]
    #byte 1 is if someone is approaching
    approaching_detected = payload[1] == 1

    expected_length = 2 + target_count * 5

    if len(payload) < expected_length:
        raise ValueError(
            f"Incomplete payload. "
            f"Expected {expected_length} bytes, "
            f"received {len(payload)}."
        )

    targets: list[RadarTarget] = []

    offset = 2

    for _ in range(target_count):

        raw_angle = payload[offset]
        distance = payload[offset + 1]
        direction = Direction(payload[offset + 2])
        speed = payload[offset + 3]
        snr = payload[offset + 4]

        target = RadarTarget(
            angle=raw_angle - 0x80,
            distance=distance,
            direction=direction,
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