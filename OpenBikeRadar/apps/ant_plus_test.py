#!/usr/bin/env python3
"""
ANT+ Bike Radar (RDR) sensor for OpenBikeRadar.

Implements the ANT+ Bike Radar Device Profile (D00001665, Rev 2.1) as a
MASTER/sensor channel, so a Garmin Edge 840 (or any ANT+ RDR-capable
display) can pair with it directly, the same way it pairs with a Garmin
Varia.

This file owns the ANT+ side only. It expects your LD2451 driver to hand
it a plain list of target dicts every cycle -- see `get_targets_from_ld2451()`
near the bottom, which is the one function you need to wire up to whatever
you already have in `ld2451/`. I couldn't fetch the contents of that folder
from GitHub (robots.txt blocks scraping the repo tree/blob pages), so I
didn't want to guess at your serial frame format and bake in something
wrong. Paste your target-reading function here, or share it with me and
I'll wire it in directly.

Requires: pip install openant pyserial
"""

import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from openant.easy.node import Node
from openant.easy.channel import Channel

# ---------------------------------------------------------------------------
# ANT+ network / channel constants (Section 5.2, Table 5-2 of the spec)
# ---------------------------------------------------------------------------

# Public ANT+ network key used throughout the open-source ANT+ ecosystem
# (e.g. openant, python-ant, Zwift-adjacent projects). If you have your own
# key from the ANT+ Adopter's Agreement, use that instead.
ANTPLUS_NETWORK_KEY = [0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45]

RF_CHANNEL_FREQUENCY = 57          # 2457 MHz (Table 5-2)
DEVICE_TYPE = 40                   # 0x28 - ANT+ Bike Radar (Table 5-2)
CHANNEL_PERIOD = 4084              # ~8 Hz (Table 5-2)
TRANSMISSION_TYPE = 0x05           # MSN=0, LSN=5 (Table 5-2 / 5.2.2)

# Data page numbers (Table 6-2)
PAGE_RADAR_TARGETS_A = 48          # 0x30
PAGE_RADAR_TARGETS_B = 49          # 0x31
PAGE_DEVICE_STATUS = 1             # 0x01
PAGE_DEVICE_COMMAND = 2            # 0x02
PAGE_MANUFACTURER_ID = 80          # 0x50 (common page)
PAGE_PRODUCT_INFO = 81             # 0x51 (common page)
PAGE_BATTERY_STATUS = 82           # 0x52 (common page)
PAGE_ERROR_DESCRIPTION = 87        # 0x57 (common page)

# Device Command values (Table 6-10)
CMD_ABORT_SHUTDOWN = 0
CMD_SHUTDOWN = 1

# Device State values (Table 6-8)
STATE_BROADCASTING = 0
STATE_SHUTDOWN_REQUESTED = 1
STATE_SHUTDOWN_ABORTED = 2
STATE_SHUTDOWN_FORCED = 3

# Threat level (Table 6-4)
THREAT_NONE = 0
THREAT_APPROACH = 1
THREAT_FAST_APPROACH = 2

# Threat side (Table 6-5)
SIDE_NONE = 0
SIDE_RIGHT = 1
SIDE_LEFT = 2

RANGE_RESOLUTION_M = 3.125          # 6 bits, 0-196.875 m (Table 6-3)
SPEED_RESOLUTION_MPS = 3.04         # 4 bits, 0-45.6 m/s  (Table 6-3)

# Manufacturer / product identity -- set these to your own values.
MANUFACTURER_ID = 255               # 255 = development/unregistered
MODEL_NUMBER = 1
HW_REVISION = 1
SW_REVISION = 1
SERIAL_NUMBER = 12345678
DEVICE_NUMBER = (SERIAL_NUMBER & 0xFFFF) or 1   # shall not be 0x0000 (5.2.3)


@dataclass
class RadarTarget:
    """One tracked vehicle target, in the units the ANT+ page expects."""
    threat_level: int          # THREAT_NONE / THREAT_APPROACH / THREAT_FAST_APPROACH
    side: int                  # SIDE_NONE / SIDE_RIGHT / SIDE_LEFT
    range_m: float             # 0-196.875 m
    closing_speed_mps: float   # 0-45.6 m/s (relative to rider, positive = approaching)


@dataclass
class RadarState:
    targets: List[RadarTarget] = field(default_factory=list)
    device_state: int = STATE_BROADCASTING
    error_active: bool = False
    error_level: int = 1
    error_code: int = 0xFF
    lock: threading.Lock = field(default_factory=threading.Lock)


# ---------------------------------------------------------------------------
# Page encoders (Section 6.5 / 6.6 / 6.7 / 6.10)
# ---------------------------------------------------------------------------

def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def encode_targets_page(page_number: int, targets: List[RadarTarget]) -> bytes:
    """Encode up to 4 targets into Data Page 48 or 49 (Table 6-3 / 6-6)."""
    targets = (targets + [None] * 4)[:4]  # pad/truncate to exactly 4 slots

    threat_levels = [0, 0, 0, 0]
    sides = [0, 0, 0, 0]
    ranges = [0, 0, 0, 0]
    speeds = [0, 0, 0, 0]

    for i, t in enumerate(targets):
        if t is None or t.threat_level == THREAT_NONE:
            continue  # fields shall be 0 for an undetected target (6.5.1-6.5.4)
        threat_levels[i] = _clamp(t.threat_level, 0, 2)
        sides[i] = _clamp(t.side, 0, 2)
        range_counts = int(round(_clamp(t.range_m, 0, 196.875) / RANGE_RESOLUTION_M))
        ranges[i] = _clamp(range_counts, 0, 0x3F)
        speed_counts = int(round(_clamp(t.closing_speed_mps, 0, 45.6) / SPEED_RESOLUTION_MPS))
        speeds[i] = _clamp(speed_counts, 0, 0x0F)

    byte1 = threat_levels[0] | (threat_levels[1] << 2) | (threat_levels[2] << 4) | (threat_levels[3] << 6)
    byte2 = sides[0] | (sides[1] << 2) | (sides[2] << 4) | (sides[3] << 6)

    # 24-bit packed range field, 6 bits per target, byte3=LSB..byte5=MSB
    packed_range = ranges[0] | (ranges[1] << 6) | (ranges[2] << 12) | (ranges[3] << 18)
    byte3 = packed_range & 0xFF
    byte4 = (packed_range >> 8) & 0xFF
    byte5 = (packed_range >> 16) & 0xFF

    byte6 = speeds[0] | (speeds[1] << 4)
    byte7 = speeds[2] | (speeds[3] << 4)

    return bytes([page_number, byte1, byte2, byte3, byte4, byte5, byte6, byte7])


def encode_device_status_page(device_state: int, clear_targets: bool = True) -> bytes:
    """Data Page 1 - Device Status (Table 6-7)."""
    byte1 = (device_state & 0x03) | 0x3F  # top 6 bits reserved -> 0x3F (Table 6-7)
    inverted_clear = 0 if clear_targets else 1  # 0 = request display clear (6.7.2)
    byte7 = (inverted_clear & 0x01) | 0x7E       # reserved bits set to 0x7F pattern shifted
    return bytes([PAGE_DEVICE_STATUS, byte1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, byte7])


def encode_error_description_page(component_index: int, error_level: int, error_code: int) -> bytes:
    """Common Page 87 - Error Description (Table 6-11)."""
    byte2 = (component_index & 0x0F) | ((error_level & 0x03) << 6)
    return bytes([PAGE_ERROR_DESCRIPTION, 0xFF, byte2, error_code & 0xFF,
                  0xFF, 0xFF, 0xFF, 0xFF])


def encode_manufacturer_id_page() -> bytes:
    """Common Page 80 (see ANT+ Common Pages doc; standard 8-byte layout)."""
    return bytes([
        PAGE_MANUFACTURER_ID, 0xFF, HW_REVISION,
        MANUFACTURER_ID & 0xFF, (MANUFACTURER_ID >> 8) & 0xFF,
        MODEL_NUMBER & 0xFF, (MODEL_NUMBER >> 8) & 0xFF, 0x00,
    ])


def encode_product_info_page() -> bytes:
    """Common Page 81."""
    sn = SERIAL_NUMBER & 0xFFFFFFFF
    return bytes([
        PAGE_PRODUCT_INFO, 0xFF, 0xFF, SW_REVISION & 0xFF,
        sn & 0xFF, (sn >> 8) & 0xFF, (sn >> 16) & 0xFF, (sn >> 24) & 0xFF,
    ])


def encode_battery_status_page(voltage: float = 3.0, status: int = 1) -> bytes:
    """Common Page 82, simplified (see ANT+ Common Pages doc for full field defs)."""
    coarse = int(voltage)
    fractional = int(round((voltage - coarse) * 256)) & 0xFF
    descriptive = ((status & 0x07) << 4) | (coarse & 0x0F)
    return bytes([PAGE_BATTERY_STATUS, 0xFF, 0xFF, 0xFF, 0xFF,
                  fractional, descriptive, 0xFF])


def encode_device_command_response_ignore() -> None:
    return None


# ---------------------------------------------------------------------------
# Transmission pattern driver (Section 6.3)
# ---------------------------------------------------------------------------

class BikeRadarSensor:
    """
    Drives the ~8 Hz broadcast pattern and implements the required
    transmission-pattern state machine (6.3.1 - 6.3.6).
    """

    def __init__(self, state: RadarState, network_num: int = 0):
        self.state = state
        self.node = Node()
        self.channel: Optional[Channel] = None
        self.network_num = network_num
        self._tick = 0
        self._background_cycle = 0   # interleave a background page every 65 pages (6.3.5)
        self._shutdown_deadline: Optional[float] = None
        self._shutdown_abort_deadline: Optional[float] = None

    # -- setup ---------------------------------------------------------

    def setup(self):
        self.node.set_network_key(self.network_num, ANTPLUS_NETWORK_KEY)
        self.channel = self.node.new_channel(Channel.Type.BIDIRECTIONAL_TRANSMIT)
        self.channel.set_id(DEVICE_NUMBER, DEVICE_TYPE, TRANSMISSION_TYPE)
        self.channel.set_period(CHANNEL_PERIOD)
        self.channel.set_rf_freq(RF_CHANNEL_FREQUENCY)
        self.channel.on_broadcast_tx_data = self._on_tx
        self.channel.on_acknowledge = self._on_acknowledge
        self.channel.open()

    # -- incoming Device Command (6.8) ---------------------------------

    def _on_acknowledge(self, data):
        """Display -> sensor acknowledged messages (Data Page 2, Device Command)."""
        if len(data) < 2 or data[0] != PAGE_DEVICE_COMMAND:
            return
        command = data[1] & 0x03
        with self.state.lock:
            if command == CMD_SHUTDOWN:
                self._begin_shutdown()
            elif command == CMD_ABORT_SHUTDOWN:
                self._abort_shutdown()

    def _begin_shutdown(self):
        # A particular radar may report Shutdown Requested or Shutdown Forced
        # depending on whether it will honor an abort (6.3.3). We choose to
        # honor aborts, so we report Shutdown Requested.
        self.state.device_state = STATE_SHUTDOWN_REQUESTED
        self._shutdown_deadline = time.time() + 2.5  # min 2.5s interleave (6.3.3)

    def _abort_shutdown(self):
        if self.state.device_state == STATE_SHUTDOWN_REQUESTED:
            self.state.device_state = STATE_SHUTDOWN_ABORTED
            self._shutdown_abort_deadline = time.time() + 10.0  # min 10s (6.3.4)
            self._shutdown_deadline = None

    # -- outgoing broadcast pattern -------------------------------------

    def _on_tx(self, data):
        self._tick += 1
        with self.state.lock:
            page = self._choose_page()
        # openant does `[channel] + data` internally, so this needs to be
        # a list of ints, not a bytes object.
        self.channel.send_broadcast_data(list(page))

    def _choose_page(self) -> bytes:
        now = time.time()

        # Shutdown / shutdown-abort states take priority (6.3.3, 6.3.4)
        if self.state.device_state == STATE_SHUTDOWN_REQUESTED:
            if self._shutdown_deadline and now >= self._shutdown_deadline:
                pass  # caller (your power-off logic) should act on this externally
            if self._tick % 2 == 0:  # ~4/8 messages, within required 1/8-4/8 range
                return encode_device_status_page(STATE_SHUTDOWN_REQUESTED)
            return self._targets_page()

        if self.state.device_state == STATE_SHUTDOWN_ABORTED:
            if self._shutdown_abort_deadline and now >= self._shutdown_abort_deadline:
                self.state.device_state = STATE_BROADCASTING
                self._shutdown_abort_deadline = None
            elif self._tick % 2 == 0:
                return encode_device_status_page(STATE_SHUTDOWN_ABORTED)
            return self._targets_page()

        # Error state (6.3.2): >= 7/8 pages must be the error page
        if self.state.error_active:
            if self._tick % 8 != 0:
                return encode_error_description_page(0x0F, self.state.error_level,
                                                       self.state.error_code)
            return self._targets_page()

        # No targets and nothing else applies (6.3.1): only Device Status,
        # requesting the display clear any shown targets.
        if not self.state.targets:
            return encode_device_status_page(STATE_BROADCASTING, clear_targets=True)

        return self._targets_page()

    def _targets_page(self) -> bytes:
        targets = self.state.targets

        # Background page interleave, only relevant when reporting <=4
        # targets (6.3.5) -- not recommended when reporting >4 (6.3.6).
        if len(targets) <= 4:
            self._background_cycle += 1
            if self._background_cycle % 65 == 0:
                idx = (self._background_cycle // 65) % 3
                return [encode_manufacturer_id_page(), encode_product_info_page(),
                        encode_battery_status_page()][idx]
            return encode_targets_page(PAGE_RADAR_TARGETS_A, targets[:4])

        # >4 targets: alternate A/B pages (6.3.6)
        if self._tick % 2 == 0:
            return encode_targets_page(PAGE_RADAR_TARGETS_A, targets[:4])
        return encode_targets_page(PAGE_RADAR_TARGETS_B, targets[4:8])

    def start(self):
        self.setup()
        self.node.start()


# ---------------------------------------------------------------------------
# LD2451 integration
#
# This assumes serial_reader.py and frame_parser.py live in an `ld2451/`
# package (frame_parser.py does `from .serial_reader import Frame`, so it
# needs to be imported as part of a package, not run standalone). Run this
# script from the repo root -- e.g. next to ld2451/ -- so the import below
# resolves. If your package is named differently, adjust the import line.
# ---------------------------------------------------------------------------

from ld2451.serial_reader import SerialReader
from ld2451.frame_parser import parse as parse_ld2451_frame, Direction, Target

# Tuning knobs -- adjust for your mounting and desired sensitivity.
ANGLE_SIDE_INVERT = False      # flip if Left/Right show up swapped on the Edge
SIDE_DEADZONE_DEG = 8          # |angle| below this -> "directly behind" (SIDE_NONE)
FAST_APPROACH_MPS = 4.0        # closing speed above this -> THREAT_FAST_APPROACH


def _target_to_radar_target(t: Target) -> Optional[RadarTarget]:
    """Convert one LD2451 Target into an ANT+ RadarTarget, or None to drop it."""
    if t.direction != Direction.APPROACHING:
        # ANT+ RDR only reports threats behind/approaching the rider.
        return None

    closing_speed = _clamp(t.closing_speed_mps, 0, 45.6)
    threat_level = THREAT_FAST_APPROACH if closing_speed > FAST_APPROACH_MPS else THREAT_APPROACH

    angle = -t.angle if ANGLE_SIDE_INVERT else t.angle
    if abs(angle) <= SIDE_DEADZONE_DEG:
        side = SIDE_NONE
    elif angle > 0:
        side = SIDE_RIGHT
    else:
        side = SIDE_LEFT

    return RadarTarget(
        threat_level=threat_level,
        side=side,
        range_m=_clamp(float(t.distance), 0, 196.875),
        closing_speed_mps=closing_speed,
    )


class LD2451Bridge:
    """
    Runs a background thread reading frames off the LD2451 via
    SerialReader, parsing them with frame_parser.parse(), and keeping the
    latest set of RadarTarget objects available for the ANT+ side to pull.
    """

    def __init__(self, port: str = "/dev/serial0", baudrate: int = 115200):
        self.reader = SerialReader(port=port, baudrate=baudrate)
        self._targets: List[RadarTarget] = []
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self.reader.open()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        for frame in self.reader:
            try:
                radar_frame = parse_ld2451_frame(frame)
            except ValueError:
                continue  # corrupt/incomplete payload, skip it

            targets = []
            for t in radar_frame.targets:
                rt = _target_to_radar_target(t)
                if rt is not None:
                    targets.append(rt)

            with self._lock:
                self._targets = targets

    def get_targets(self) -> List[RadarTarget]:
        with self._lock:
            return list(self._targets)

    def stop(self):
        self.reader.close()


def polling_loop(state: RadarState, bridge: LD2451Bridge, poll_hz: float = 10.0):
    period = 1.0 / poll_hz
    while True:
        targets = bridge.get_targets()
        with state.lock:
            state.targets = targets[:8]
        time.sleep(period)


def main():
    state = RadarState()
    sensor = BikeRadarSensor(state)

    bridge = LD2451Bridge(port="/dev/serial0", baudrate=115200)
    bridge.start()

    poll_thread = threading.Thread(target=polling_loop, args=(state, bridge), daemon=True)
    poll_thread.start()

    try:
        sensor.start()  # blocks, running the ANT+ event loop
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        sensor.node.stop()


if __name__ == "__main__":
    main()