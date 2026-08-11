"""
ANT+ Bike Radar broadcaster (master / sensor side).

Uses openant to appear as an ANT+ Bike Radar (device type 40).
"""

from __future__ import annotations

import logging
import threading
import time
from typing import List, Optional, Sequence

from openant.easy.node import Node
from openant.easy.channel import Channel

from .pages import (
    build_page_48,
    build_page_1_device_status,
    build_page_80_manufacturer,
    build_page_81_product,
)
from .threat import AntTarget

logger = logging.getLogger(__name__)

# ANT+ public network key
ANTPLUS_NETWORK_KEY = [0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45]

DEVICE_TYPE = 40          # 0x28 – Bike Radar
CHANNEL_PERIOD = 4084     # ~8 Hz
RF_FREQUENCY = 57         # 2457 MHz
TRANSMISSION_TYPE = 5     # LSN = 5 as required by the profile


class AntRadarBroadcaster:
    """
    Broadcasts tracked targets as an ANT+ Bike Radar.

    Typical usage:

        broadcaster = AntRadarBroadcaster(device_number=12345)
        broadcaster.start()

        # in your main loop / tracker callback:
        broadcaster.update_targets(list_of_AntTarget)

        # later:
        broadcaster.stop()
    """

    def __init__(
        self,
        device_number: int = 54321,
        manufacturer_id: int = 0x00FF,   # development / open-source
        model_number: int = 1,
        software_version: int = 1,
        serial_number: int = 0x12345678,
    ):
        if not (1 <= device_number <= 65535):
            raise ValueError("device_number must be 1–65535")

        self.device_number = device_number
        self.manufacturer_id = manufacturer_id
        self.model_number = model_number
        self.software_version = software_version
        self.serial_number = serial_number

        self._node: Optional[Node] = None
        self._channel: Optional[Channel] = None

        self._lock = threading.Lock()
        self._targets: List[AntTarget] = []
        self._page_counter = 0
        self._running = False
        self._tx_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update_targets(self, targets: Sequence[AntTarget]) -> None:
        """Replace the current list of targets (thread-safe)."""
        with self._lock:
            self._targets = list(targets)[:8]   # profile supports max 8

    def start(self) -> None:
        if self._running:
            return

        logger.info(
            "Starting ANT+ Bike Radar (device_number=%s, type=%s)",
            self.device_number,
            DEVICE_TYPE,
        )

        self._node = Node()
        self._node.set_network_key(0x00, ANTPLUS_NETWORK_KEY)

        self._channel = self._node.new_channel(
            Channel.Type.BIDIRECTIONAL_TRANSMIT, 0x00, 0x00
        )
        self._channel.set_id(self.device_number, DEVICE_TYPE, TRANSMISSION_TYPE)
        self._channel.set_period(CHANNEL_PERIOD)
        self._channel.set_rf_freq(RF_FREQUENCY)

        # Optional: hook the TX event if you want exact timing later
        # self._channel.on_broadcast_tx_data = self._on_tx_event

        self._channel.open()
        self._running = True

        # Simple timed broadcaster (good enough for first version)
        self._tx_thread = threading.Thread(
            target=self._broadcast_loop, daemon=True, name="ant-radar-tx"
        )
        self._tx_thread.start()

        logger.info("ANT+ Radar broadcaster running")

    def stop(self) -> None:
        self._running = False

        if self._tx_thread and self._tx_thread.is_alive():
            self._tx_thread.join(timeout=2.0)

        if self._channel:
            try:
                self._channel.close()
            except Exception:
                pass
            self._channel = None

        if self._node:
            try:
                self._node.stop()
            except Exception:
                pass
            self._node = None

        logger.info("ANT+ Radar broadcaster stopped")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _broadcast_loop(self) -> None:
        """Send a page roughly every 125 ms (~8 Hz)."""
        interval = 4084 / 32768.0          # exact channel period in seconds

        while self._running:
            start = time.monotonic()

            try:
                payload = self._next_payload()
                if self._channel and self._running:
                    self._channel.send_broadcast_data(payload)
            except Exception as e:
                logger.warning("Broadcast error: %s", e)

            # Maintain approximate 8 Hz
            elapsed = time.monotonic() - start
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _next_payload(self) -> list[int]:
        """
        Decide which page to send this tick.

        Simple pattern:
          - Most of the time → Page 48 (targets 1-4)
          - Every 65th message → background page (80 or 81)
        """
        with self._lock:
            targets = list(self._targets)
            counter = self._page_counter
            self._page_counter += 1

        # Background pages roughly once every 65 messages
        if counter % 65 == 0:
            return list(build_page_80_manufacturer(
                manufacturer_id=self.manufacturer_id,
                model_number=self.model_number,
            ))
        elif counter % 65 == 32:
            return list(build_page_81_product(
                software_version=self.software_version,
                serial_number=self.serial_number,
            ))

        # Main data: Page 48 (first 4 targets)
        # (Page 49 support can be added later if you ever have >4 targets)
        page = build_page_48(targets[:4])
        return list(page)


# ----------------------------------------------------------------------
# Convenience helper – turn your tracker targets into AntTargets
# ----------------------------------------------------------------------

def tracked_to_ant_targets(tracked_targets) -> List[AntTarget]:
    """
    Convert objects from your tracker package into AntTarget instances.

    Expected attributes on each tracked target:
        .distance          (meters)
        .closing_speed_mps or .speed (km/h or m/s – adjust as needed)
        .angle             (degrees)
        .direction         (your Direction enum)
    """
    from .threat import (
        AntTarget,
        make_threat_level,
        make_threat_side,
        ThreatLevel,
    )

    result = []
    for t in tracked_targets:
        # Adjust these attribute names to match your actual tracker Target
        distance = getattr(t, "distance", 0.0)
        angle = getattr(t, "angle", 0.0)

        # Prefer a real closing speed if you have it
        if hasattr(t, "closing_speed_mps"):
            speed_mps = t.closing_speed_mps
        elif hasattr(t, "speed"):
            # assume km/h → m/s
            speed_mps = t.speed / 3.6
        else:
            speed_mps = 0.0

        is_approaching = True
        if hasattr(t, "direction"):
            # Adjust according to your Direction enum
            is_approaching = str(t.direction).upper().endswith("APPROACHING")

        level = make_threat_level(speed_mps, is_approaching=is_approaching)
        side = make_threat_side(angle)

        result.append(
            AntTarget(
                threat_level=level,
                threat_side=side,
                range_m=float(distance),
                closing_speed_mps=float(speed_mps),
            )
        )

    return result