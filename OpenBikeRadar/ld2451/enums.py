from enum import Enum
class Direction(Enum):
    APPROACHING = 1
    MOVING_AWAY = 0

    def __str__(self):
        return self.name.replace("_", " ").title()