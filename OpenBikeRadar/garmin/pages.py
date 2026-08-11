"""
Builders for ANT+ Bike Radar data pages (sensor / master side).
"""

from __future__ import annotations
from typing import Sequence, List, Optional
from .threat import AntTarget, ThreatLevel, quantize_range, quantize_closing_speed


def build_page_48(targets: Sequence[AntTarget]) -> bytes:
    """
    Data Page 48 – Radar Targets A (targets 1-4).

    Always returns exactly 8 bytes.
    Missing targets are encoded as No Threat / 0 range / 0 speed.
    """
    # Pad / truncate to exactly 4 slots
    slots: List[Optional[AntTarget]] = list(targets[:4])
    while len(slots) < 4:
        slots.append(None)

    # Byte 1 – Threat Levels (2 bits each)
    threat_byte = 0
    for i, t in enumerate(slots):
        level = t.threat_level if t is not None else ThreatLevel.NO_THREAT
        threat_byte |= (int(level) & 0x03) << (i * 2)

    # Byte 2 – Threat Sides (2 bits each)
    side_byte = 0
    for i, t in enumerate(slots):
        side = t.threat_side if t is not None else 0
        side_byte |= (int(side) & 0x03) << (i * 2)

    # Bytes 3-5 – Ranges (6 bits each, packed into 24 bits)
    ranges = []
    for t in slots:
        if t is None or t.threat_level == ThreatLevel.NO_THREAT:
            ranges.append(0)
        else:
            ranges.append(quantize_range(t.range_m))

    # Pack four 6-bit values into three bytes (little-endian bit packing)
    range_bits = (
        (ranges[0] & 0x3F) |
        ((ranges[1] & 0x3F) << 6) |
        ((ranges[2] & 0x3F) << 12) |
        ((ranges[3] & 0x3F) << 18)
    )
    range_bytes = [
        (range_bits >> 0) & 0xFF,
        (range_bits >> 8) & 0xFF,
        (range_bits >> 16) & 0xFF,
    ]

    # Bytes 6-7 – Closing Speeds (4 bits each)
    speeds = []
    for t in slots:
        if t is None or t.threat_level == ThreatLevel.NO_THREAT:
            speeds.append(0)
        else:
            speeds.append(quantize_closing_speed(t.closing_speed_mps))

    speed_byte0 = (speeds[0] & 0x0F) | ((speeds[1] & 0x0F) << 4)
    speed_byte1 = (speeds[2] & 0x0F) | ((speeds[3] & 0x0F) << 4)

    return bytes([
        0x30,               # page number
        threat_byte,
        side_byte,
        range_bytes[0],
        range_bytes[1],
        range_bytes[2],
        speed_byte0,
        speed_byte1,
    ])


def build_page_49(targets: Sequence[AntTarget]) -> bytes:
    """
    Data Page 49 – Radar Targets B (targets 5-8).
    Identical layout to page 48, just a different page number.
    """
    page = bytearray(build_page_48(targets))
    page[0] = 0x31
    return bytes(page)


def build_page_1_device_status(
    device_state: int = 0,          # 0 = Broadcasting
    clear_targets: bool = False,
) -> bytes:
    """
    Data Page 1 – Device Status.

    device_state:
        0 = Broadcasting
        1 = Shutdown Requested
        2 = Shutdown Aborted
        3 = Shutdown Forced
    """
    # Byte 7 bit 0 is *inverted* clear flag
    # 0 = request clear, 1 = no action
    clear_bit = 0 if clear_targets else 1

    return bytes([
        0x01,                       # page number
        (device_state & 0x03),      # bits 0-1
        0x3F,                       # reserved (was 0x00 on some legacy)
        0xFF, 0xFF, 0xFF, 0xFF,     # reserved
        (clear_bit & 0x01) | 0x7E,  # bit 0 + reserved 0x7F-ish
    ])


def build_page_80_manufacturer(
    manufacturer_id: int = 0x00FF,   # 0x00FF = development / open source
    model_number: int = 1,
    hw_revision: int = 1,
) -> bytes:
    """Common Page 80 – Manufacturer’s Identification."""
    return bytes([
        0x50,
        0xFF,                       # reserved
        hw_revision & 0xFF,
        manufacturer_id & 0xFF,
        (manufacturer_id >> 8) & 0xFF,
        model_number & 0xFF,
        (model_number >> 8) & 0xFF,
        0xFF,                       # reserved
    ])


def build_page_81_product(
    software_version: int = 1,
    serial_number: int = 0x12345678,
) -> bytes:
    """Common Page 81 – Product Information."""
    return bytes([
        0x51,
        0xFF,                       # reserved
        software_version & 0xFF,
        (serial_number >> 0) & 0xFF,
        (serial_number >> 8) & 0xFF,
        (serial_number >> 16) & 0xFF,
        (serial_number >> 24) & 0xFF,
        0xFF,                       # reserved
    ])