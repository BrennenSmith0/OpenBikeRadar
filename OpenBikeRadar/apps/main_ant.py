"""
End-to-end application:
LD2451 → SerialReader → FrameParser → Tracker → ANT+ Bike Radar
"""

from __future__ import annotations

import logging
import signal
import sys
import time
from pathlib import Path

# Make the project root importable when running as a script
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config
from ld2451.serial_reader import SerialReader
from ld2451.frame_parser import parse
from ld2451.enums import Direction

from tracker.tracker import Tracker

from garmin.ant_radar import AntRadarBroadcaster
from garmin.threat import (
    AntTarget,
    make_threat_level,
    make_threat_side,
)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("ant_main")


def tracked_to_ant(targets) -> list[AntTarget]:
    """
    Convert TrackedTarget objects into AntTarget for the broadcaster.

    - speed on TrackedTarget is treated as km/h (same as LD2451)
    - closing_speed_mps is signed: positive = approaching, negative = receding
    - has_approached comes from the tracker
    """
    ant_targets: list[AntTarget] = []

    for t in targets:
        distance_m = float(t.distance)
        angle = float(t.angle)

        # km/h → m/s, then apply sign from direction
        speed_mps = float(t.speed) / 3.6
        if t.direction == Direction.MOVING_AWAY:
            speed_mps = -speed_mps

        level = make_threat_level(
            closing_speed_mps=speed_mps,
            range_m=distance_m,
            has_approached=getattr(t, "has_approached", False),
        )
        side = make_threat_side(angle)

        ant_targets.append(
            AntTarget(
                threat_level=level,
                threat_side=side,
                range_m=distance_m,
                closing_speed_mps=speed_mps,
            )
        )

    return ant_targets


def main() -> None:
    logger.info("Starting OpenBikeRadar (ANT+)")
    logger.info("Device number = %s", config.ANT_DEVICE_NUMBER)

    broadcaster = AntRadarBroadcaster(device_number=config.ANT_DEVICE_NUMBER)
    broadcaster.start()

    tracker = Tracker()

    stop = False

    def handle_sigint(sig, frame):
        nonlocal stop
        logger.info("Ctrl-C received – shutting down…")
        stop = True

    signal.signal(signal.SIGINT, handle_sigint)

    frame_count = 0
    last_time = time.monotonic()
    last_print = last_time

    try:
        with SerialReader() as reader:
            logger.info("Serial reader opened – waiting for frames…")

            for frame in reader:
                if stop:
                    break

                now = time.monotonic()
                dt = now - last_time
                last_time = now

                # Guard against huge dt after pauses / first frame
                if dt <= 0 or dt > 1.0:
                    dt = 0.1

                radar = parse(frame)
                frame_count += 1

                # Track across frames (sets has_approached, stable IDs, etc.)
                tracked = tracker.update(radar.targets, dt)

                # Convert and broadcast
                ant_targets = tracked_to_ant(tracked)
                broadcaster.update_targets(ant_targets)

                # Console feedback ~1 Hz
                if now - last_print >= 1.0:
                    last_print = now
                    if not tracked:
                        logger.info("No tracked targets")
                    else:
                        for t in tracked:
                            sign = "+" if t.direction == Direction.APPROACHING else "-"
                            logger.info(
                                "  id=%d  %s%.0fkm/h  %dm  %+d°  approached=%s  missed=%d",
                                t.id,
                                sign,
                                t.speed,
                                t.distance,
                                t.angle,
                                t.has_approached,
                                t.missed_frames,
                            )

    except Exception as e:
        logger.exception("Fatal error: %s", e)
    finally:
        logger.info("Stopping broadcaster…")
        broadcaster.stop()
        logger.info("Done. Frames processed: %d", frame_count)


if __name__ == "__main__":
    main()