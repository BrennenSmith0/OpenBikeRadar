from ld2451.enums import Direction
from dataclasses import dataclass

@dataclass
class TrackedTarget:
    id: int

    distance: float
    angle: int
    speed: float
    direction: Direction
    snr: int

    age: int = 1
    missed_frames: int = 0
    has_approached: bool = False          # ← new