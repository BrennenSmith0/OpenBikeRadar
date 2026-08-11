#!/usr/bin/env python3
"""
End-to-end test: LD2451 → tracker → ANT+ Bike Radar broadcast.

Requirements:
  - ANT USB stick plugged in
  - openant installed
  - LD2451 connected and powered
"""

from __future__ import annotations

import logging
import signal
import sys
import time
from pathlib import Path

# Make the project root importable when running as a script
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ld2451.serial_reader import SerialReader
from ld2451.frame_parser import parse
from ld2451.frame_parser import Direction  # adjust if your enum lives elsewhere

from garmin.ant_radar import AntRadarBroadcaster
from garmin.threat import (
    AntTarget,
    ThreatLevel,
    ThreatSide,
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
logger = logging.getLogger("ant_radar_test")

# ---------------------------------------------------------------------------
# Configuration – tweak these
# ---------------------------------------------------------------------------
ANT_DEVICE_NUMBER = 12345          # Change if you want a different ID
SERIAL_PORT = None                 # None = auto / default from SerialReader
PRINT_EVERY_N_FRAMES = 10          # How often to print status to console


def radar_targets_to_ant(targets) -> list[AntTarget]:
    """
    Convert your LD2451 / tracker Target objects into AntTarget.

    Adjust the attribute names below to match whatever your
    tracker (or frame_parser.Target) actually exposes.
    """
    ant_targets = []

    for t in targets:
        # --- distance ---
        distance_m = float(getattr(t, "distance", 0))

        # --- closing speed (m/s) ---
        if hasattr(t, "closing_speed_mps"):
            speed_mps = float(t.closing_speed_mps)
        elif hasattr(t, "speed"):
            # assume km/h → m/s
            speed_mps = float(t.speed) / 3.6
        else:
            speed_mps = 0.0

        # --- approaching? ---
        is_approaching = True
        direction = getattr(t, "direction", None)
        if direction is not None:
            # Works with both Enum and string
            dir_str = str(direction).upper()
            is_approaching = "APPROACH" in dir_str

        # --- angle → side ---
        angle = float(getattr(t, "angle", 0))

        level = make_threat_level(
            closing_speed_mps=speed_mps,
            range_m=distance_m,
            has_approached=getattr(t, "has_approached", True),
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
    logger.info("Starting ANT+ Bike Radar test")
    logger.info("Device number = %s", ANT_DEVICE_NUMBER)

    broadcaster = AntRadarBroadcaster(device_number=ANT_DEVICE_NUMBER)
    broadcaster.start()

    # Graceful shutdown on Ctrl-C
    stop = False

    def handle_sigint(sig, frame):
        nonlocal stop
        logger.info("Ctrl-C received – shutting down…")
        stop = True

    signal.signal(signal.SIGINT, handle_sigint)

    frame_count = 0
    last_print = time.monotonic()

    try:
        with SerialReader() as reader:          # uses default port / settings
            logger.info("Serial reader opened – waiting for frames…")

            for frame in reader:
                if stop:
                    break

                radar = parse(frame)
                frame_count += 1

                # Convert and push to ANT+
                ant_targets = radar_targets_to_ant(radar.targets)
                broadcaster.update_targets(ant_targets)

                # Occasional console feedback
                now = time.monotonic()
                if now - last_print >= 1.0:     # roughly once per second
                    last_print = now
                    if radar.target_count == 0:
                        logger.info("No targets")
                    else:
                        for i, t in enumerate(radar.targets):
                            logger.info(
                                "  Target %d: %s",
                                i + 1,
                                t,               # uses your Target.__str__
                            )

    except Exception as e:
        logger.exception("Fatal error: %s", e)
    finally:
        logger.info("Stopping broadcaster…")
        broadcaster.stop()
        logger.info("Done. Frames processed: %d", frame_count)


if __name__ == "__main__":
    main()