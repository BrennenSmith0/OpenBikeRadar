"""
Helpers for mapping tracked radar targets into ANT+ Bike Radar fields.

More Varia-like behaviour:
- Fast approaching          → Vehicle Fast Approach
- Approaching or holding    → Vehicle Approach
- Slowly drifting back      → still shown as Vehicle Approach
- Clearly pulling away      → No Threat
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
    range_m: float              # meters
    closing_speed_mps: float    # relative speed (positive = approaching)


# ---------------------------------------------------------------------------
# Tunable thresholds (tweak these to taste)
# ---------------------------------------------------------------------------

# Relative speed above this → Fast Approach
FAST_APPROACH_MPS = 8.0          # ≈ 29 km/h relative

# Relative speed above this → still treated as a threat
# (allows matching speed and slowly drifting back)
MIN_THREAT_MPS = -2.5            # ≈ -9 km/h relative

# Beyond this distance we become stricter about showing receding targets
FAR_RANGE_M = 60.0


def make_threat_level(
    closing_speed_mps: float,
    range_m: float = 0.0,
    has_approached: bool = True,      # ← new parameter
) -> ThreatLevel:

    # Never promote pure-receding noise
    if not has_approached:
        return ThreatLevel.NO_THREAT

    # … rest of the logic we already have …
    if closing_speed_mps >= FAST_APPROACH_MPS:
        return ThreatLevel.VEHICLE_FAST_APPROACH

    if closing_speed_mps >= MIN_THREAT_MPS:
        return ThreatLevel.VEHICLE_APPROACH

    if range_m < 25.0 and closing_speed_mps > -5.0:
        return ThreatLevel.VEHICLE_APPROACH

    return ThreatLevel.NO_THREAT


def make_threat_side(angle_deg: float) -> ThreatSide:
    """
    angle_deg from your LD2451 parser.
    Positive / negative depends on your mounting orientation.
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
    """
    Convert m/s → 4-bit value (units of 3.04 m/s).

    Note: the ANT+ field is unsigned. We only send the magnitude
    of the closing speed. Direction is already captured by threat level.
    """
    speed = abs(mps)
    if speed <= 0:
        return 0
    value = round(speed / 3.04)
    return max(0, min(15, value))           # 0 … 45.6 m/s