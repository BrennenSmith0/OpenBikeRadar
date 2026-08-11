"""
Helpers for mapping tracked radar targets into ANT+ Bike Radar fields.
"""

from __future__ import annotations
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional


class ThreatLevel(IntEnum):
    NO_THREAT = 0
    VEHICLE_APPROACH = 1
    VEHICLE_FAST_APPROACH = 2
    # 3 is reserved


class ThreatSide(IntEnum):
    BEHIND = 0          # No side / directly behind
    RIGHT = 1
    LEFT = 2
    # 3 is reserved


@dataclass(frozen=True)
class AntTarget:
    """Minimal data needed to encode one ANT+ radar target."""
    threat_level: ThreatLevel
    threat_side: ThreatSide
    range_m: float          # meters
    closing_speed_mps: float  # relative speed toward the rider (positive = approaching)


def make_threat_level(closing_speed_mps: float, is_approaching: bool = True) -> ThreatLevel:
    """
    Simple heuristic. Tweak the thresholds to taste.
    """
    if not is_approaching or closing_speed_mps <= 0:
        return ThreatLevel.NO_THREAT

    if closing_speed_mps < 8.0:          # ~29 km/h relative
        return ThreatLevel.VEHICLE_APPROACH
    else:
        return ThreatLevel.VEHICLE_FAST_APPROACH


def make_threat_side(angle_deg: float) -> ThreatSide:
    """
    angle_deg from your LD2451 parser:
      positive = one side, negative = the other.
    Adjust the sign convention if your mounting is mirrored.
    """
    if abs(angle_deg) < 8.0:
        return ThreatSide.BEHIND
    elif angle_deg > 0:
        return ThreatSide.RIGHT
    else:
        return ThreatSide.LEFT


def quantize_range(meters: float) -> int:
    """Convert meters → 6-bit value (units of 3.125 m)."""
    if meters <= 0:
        return 0
    value = round(meters / 3.125)
    return max(0, min(63, value))           # 0 … 196.875 m


def quantize_closing_speed(mps: float) -> int:
    """Convert m/s → 4-bit value (units of 3.04 m/s)."""
    if mps <= 0:
        return 0
    value = round(mps / 3.04)
    return max(0, min(15, value))           # 0 … 45.6 m/s