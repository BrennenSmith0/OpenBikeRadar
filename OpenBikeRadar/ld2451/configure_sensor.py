"""
configure.py

Configuration interface for the HLK-LD2451 radar.

This module handles the LD2451 command/ACK protocol.

Command protocol:

    FD FC FB FA
    2-byte little-endian length
    command + value
    04 03 02 01

This is different from the radar target-reporting protocol used by
serial_reader.py:

    F4 F3 F2 F1
    2-byte little-endian length
    radar data
    F8 F7 F6 F5
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import argparse
import struct
import time

import serial
import config


# ---------------------------------------------------------------------------
# Protocol framing
# ---------------------------------------------------------------------------

FRAME_HEADER = b"\xFD\xFC\xFB\xFA"
FRAME_TAIL = b"\x04\x03\x02\x01"


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

CMD_ENABLE_CONFIG = 0x00FF
CMD_END_CONFIG = 0x00FE

CMD_SET_DETECTION_PARAMS = 0x0002
CMD_SET_SENSITIVITY = 0x0003

CMD_READ_DETECTION_PARAMS = 0x0012
CMD_READ_SENSITIVITY = 0x0013

CMD_READ_FIRMWARE = 0x00A0
CMD_SET_BAUDRATE = 0x00A1
CMD_FACTORY_RESET = 0x00A2
CMD_RESTART = 0x00A3


# ---------------------------------------------------------------------------
# Configuration values
# ---------------------------------------------------------------------------

DIRECTION_AWAY = 0x00
DIRECTION_APPROACH = 0x01
DIRECTION_ALL = 0x02

DIRECTION_NAMES = {
    DIRECTION_AWAY: "away",
    DIRECTION_APPROACH: "approach",
    DIRECTION_ALL: "all",
}

BAUDRATE_TO_INDEX = {
    9600: 0x0001,
    19200: 0x0002,
    38400: 0x0003,
    57600: 0x0004,
    115200: 0x0005,
    230400: 0x0006,
    256000: 0x0007,
    460800: 0x0008,
}

INDEX_TO_BAUDRATE = {
    value: key
    for key, value in BAUDRATE_TO_INDEX.items()
}


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class DetectionParameters:
    """LD2451 target detection parameters."""

    max_distance: int
    direction: int
    min_speed: int
    delay: int

    @property
    def direction_name(self) -> str:
        return DIRECTION_NAMES.get(
            self.direction,
            f"unknown(0x{self.direction:02X})",
        )

    def __str__(self) -> str:
        return (
            f"max_distance={self.max_distance}m, "
            f"direction={self.direction_name}, "
            f"min_speed={self.min_speed}km/h, "
            f"delay={self.delay}s"
        )


@dataclass(frozen=True)
class Sensitivity:
    """
    LD2451 sensitivity configuration.

    trigger_count:
        Number of consecutive detections required before reporting.

    snr_threshold:
        Signal-to-noise ratio threshold.

    extended_1 / extended_2:
        Extended sensitivity parameters documented by the sensor.
    """

    trigger_count: int
    snr_threshold: int
    extended_1: int
    extended_2: int

    def __str__(self) -> str:
        return (
            f"trigger_count={self.trigger_count}, "
            f"snr_threshold={self.snr_threshold}, "
            f"extended_1={self.extended_1}, "
            f"extended_2={self.extended_2}"
        )


@dataclass(frozen=True)
class FirmwareVersion:
    """Decoded LD2451 firmware information."""

    radar_type: int
    host_version: int
    minor_version: int

    def __str__(self) -> str:
        # The protocol example reports:
        #
        # 51 24 -> 0x2451
        # 01 01
        # 10 15 05 24
        #
        # as V1.01.24051510.
        #
        # Keep the raw fields available rather than making assumptions
        # beyond the documented representation.
        return (
            f"type=0x{self.radar_type:04X}, "
            f"host=0x{self.host_version:04X}, "
            f"minor=0x{self.minor_version:08X}"
        )


# ---------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------

def build_frame(
    command: int,
    value: bytes = b"",
) -> bytes:
    """
    Build an LD2451 configuration command frame.
    """

    data = struct.pack("<H", command) + value
    length = struct.pack("<H", len(data))

    return (
        FRAME_HEADER
        + length
        + data
        + FRAME_TAIL
    )


# ---------------------------------------------------------------------------
# Configurator
# ---------------------------------------------------------------------------

class LD2451Configurator:
    """
    Communicates with the LD2451 configuration interface.

    Example:

        with LD2451Configurator() as radar:
            print(radar.read_parameters())
            print(radar.read_sensitivity())
            print(radar.read_firmware())
    """

    def __init__(
        self,
        port: str = config.SERIAL_PORT,
        baudrate: int = config.BAUDRATE,
        timeout: float = 1.0,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.ser: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # Serial handling
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Open the serial connection."""

        if self.ser is None:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
            )

        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        """Close the serial connection."""

        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "LD2451Configurator":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Low-level protocol
    # ------------------------------------------------------------------

    def _read_frame(
        self,
        timeout: float = 2.0,
    ) -> bytes:
        """
        Read one complete configuration ACK frame.
        """

        if self.ser is None:
            raise RuntimeError(
                "Serial port has not been opened."
            )

        deadline = time.monotonic() + timeout
        buffer = bytearray()

        while time.monotonic() < deadline:

            if self.ser.in_waiting:
                buffer.extend(
                    self.ser.read(self.ser.in_waiting)
                )

                start = buffer.find(FRAME_HEADER)

                if start == -1:
                    continue

                # Need header + length.
                if len(buffer) < start + 6:
                    continue

                length = struct.unpack_from(
                    "<H",
                    buffer,
                    start + 4,
                )[0]

                total_length = (
                    4
                    + 2
                    + length
                    + 4
                )

                if len(buffer) < start + total_length:
                    continue

                frame = bytes(
                    buffer[
                        start:start + total_length
                    ]
                )

                if frame.endswith(FRAME_TAIL):
                    return frame

                # Invalid frame. Continue searching.
                del buffer[:start + 4]

            else:
                time.sleep(0.005)

        raise TimeoutError(
            "Timed out waiting for LD2451 ACK."
        )

    def _send_command(
        self,
        command: int,
        value: bytes = b"",
        timeout: float = 2.0,
    ) -> bytes:
        """
        Send a command and return its ACK payload.

        The returned bytes exclude:

            ACK command word
            ACK status

        so callers receive only the command-specific data.
        """

        if self.ser is None:
            raise RuntimeError(
                "Serial port has not been opened."
            )

        frame = build_frame(command, value)

        print(
            f"TX [{command:#06x}]: "
            f"{frame.hex(' ').upper()}"
        )

        self.ser.write(frame)

        response = self._read_frame(timeout)

        print(
            f"RX [{command:#06x}]: "
            f"{response.hex(' ').upper()}"
        )

        length = struct.unpack_from(
            "<H",
            response,
            4,
        )[0]

        payload = response[6:6 + length]

        if len(payload) < 4:
            raise RuntimeError(
                "Malformed LD2451 ACK."
            )

        ack_command = struct.unpack_from(
            "<H",
            payload,
            0,
        )[0]

        status = struct.unpack_from(
            "<H",
            payload,
            2,
        )[0]

        expected_ack = command | 0x0100

        if ack_command != expected_ack:
            raise RuntimeError(
                f"Unexpected ACK command: "
                f"expected 0x{expected_ack:04X}, "
                f"received 0x{ack_command:04X}"
            )

        if status != 0:
            raise RuntimeError(
                f"LD2451 command 0x{command:04X} "
                f"failed with status {status}"
            )

        return payload[4:]

    # ------------------------------------------------------------------
    # Configuration session
    # ------------------------------------------------------------------

    def enable(self) -> bytes:
        """Enter LD2451 configuration mode."""

        return self._send_command(
            CMD_ENABLE_CONFIG,
            struct.pack("<H", 0x0001),
        )

    def end(self) -> bytes:
        """Exit LD2451 configuration mode."""

        return self._send_command(
            CMD_END_CONFIG
        )

    # ------------------------------------------------------------------
    # Detection parameters
    # ------------------------------------------------------------------

    def read_parameters(self) -> DetectionParameters:
        """
        Read the current radar detection parameters.
        """

        self.enable()

        try:
            data = self._send_command(
                CMD_READ_DETECTION_PARAMS
            )
        finally:
            self.end()

        if len(data) < 4:
            raise RuntimeError(
                "Invalid detection parameter response."
            )

        return DetectionParameters(
            max_distance=data[0],
            direction=data[1],
            min_speed=data[2],
            delay=data[3],
        )

    def set_parameters(
        self,
        max_distance: int,
        direction: int = DIRECTION_ALL,
        min_speed: int = 0,
        delay: int = 2,
    ) -> DetectionParameters:
        """
        Set radar target detection parameters.

        max_distance:
            10-255 meters according to the protocol.

        direction:
            0 = away
            1 = approaching
            2 = both

        min_speed:
            0-120 km/h.

        delay:
            No-target delay in seconds.
        """

        if not 10 <= max_distance <= 255:
            raise ValueError(
                "max_distance must be between 10 and 255."
            )

        if direction not in (
            DIRECTION_AWAY,
            DIRECTION_APPROACH,
            DIRECTION_ALL,
        ):
            raise ValueError(
                "Invalid direction."
            )

        if not 0 <= min_speed <= 120:
            raise ValueError(
                "min_speed must be between 0 and 120 km/h."
            )

        if not 0 <= delay <= 255:
            raise ValueError(
                "delay must be between 0 and 255 seconds."
            )

        value = bytes(
            [
                max_distance,
                direction,
                min_speed,
                delay,
            ]
        )

        self.enable()

        try:
            self._send_command(
                CMD_SET_DETECTION_PARAMS,
                value,
            )
        finally:
            self.end()

        return self.read_parameters()

    # ------------------------------------------------------------------
    # Sensitivity
    # ------------------------------------------------------------------

    def read_sensitivity(self) -> Sensitivity:
        """Read the current radar sensitivity."""

        self.enable()

        try:
            data = self._send_command(
                CMD_READ_SENSITIVITY
            )
        finally:
            self.end()

        if len(data) < 4:
            raise RuntimeError(
                "Invalid sensitivity response."
            )

        return Sensitivity(
            trigger_count=data[0],
            snr_threshold=data[1],
            extended_1=data[2],
            extended_2=data[3],
        )

    def set_sensitivity(
        self,
        trigger_count: int,
        snr_threshold: int,
        extended_1: int = 0,
        extended_2: int = 0,
    ) -> Sensitivity:
        """
        Set radar sensitivity.

        The protocol defines:
            byte 0 = cumulative effective trigger count
            byte 1 = SNR threshold
            byte 2 = extended parameter
            byte 3 = extended parameter
        """

        if not 1 <= trigger_count <= 10:
            raise ValueError(
                "trigger_count must be between 1 and 10."
            )

        if not 0 <= snr_threshold <= 255:
            raise ValueError(
                "snr_threshold must be between 0 and 255."
            )

        if not 0 <= extended_1 <= 255:
            raise ValueError(
                "extended_1 must be between 0 and 255."
            )

        if not 0 <= extended_2 <= 255:
            raise ValueError(
                "extended_2 must be between 0 and 255."
            )

        value = bytes(
            [
                trigger_count,
                snr_threshold,
                extended_1,
                extended_2,
            ]
        )

        self.enable()

        try:
            self._send_command(
                CMD_SET_SENSITIVITY,
                value,
            )
        finally:
            self.end()

        return self.read_sensitivity()

    # ------------------------------------------------------------------
    # Firmware
    # ------------------------------------------------------------------

    def read_firmware(self) -> FirmwareVersion:
        """Read firmware information from the radar."""

        self.enable()

        try:
            data = self._send_command(
                CMD_READ_FIRMWARE
            )
        finally:
            self.end()

        if len(data) < 8:
            raise RuntimeError(
                "Invalid firmware response."
            )

        radar_type = struct.unpack_from(
            "<H",
            data,
            0,
        )[0]

        host_version = struct.unpack_from(
            "<H",
            data,
            2,
        )[0]

        minor_version = struct.unpack_from(
            "<I",
            data,
            4,
        )[0]

        return FirmwareVersion(
            radar_type=radar_type,
            host_version=host_version,
            minor_version=minor_version,
        )

    # ------------------------------------------------------------------
    # Baud rate
    # ------------------------------------------------------------------

    def set_baudrate(
        self,
        baudrate: int,
    ) -> None:
        """
        Set the radar's serial baud rate.

        The new baud rate takes effect after the radar is restarted.
        """

        if baudrate not in BAUDRATE_TO_INDEX:
            valid = ", ".join(
                str(value)
                for value in BAUDRATE_TO_INDEX
            )

            raise ValueError(
                f"Unsupported baud rate. "
                f"Supported values: {valid}"
            )

        index = BAUDRATE_TO_INDEX[baudrate]

        self.enable()

        try:
            self._send_command(
                CMD_SET_BAUDRATE,
                struct.pack("<H", index),
            )
        finally:
            self.end()

    # ------------------------------------------------------------------
    # Factory reset
    # ------------------------------------------------------------------

    def factory_reset(self) -> None:
        """
        Restore the radar's configuration to factory defaults.

        The protocol states that factory-reset values take effect
        after the radar is restarted.
        """

        self.enable()

        try:
            self._send_command(
                CMD_FACTORY_RESET
            )
        finally:
            self.end()

    # ------------------------------------------------------------------
    # Restart
    # ------------------------------------------------------------------

    def restart(self) -> None:
        """
        Restart the radar.

        The radar sends its ACK before restarting.
        """

        self.enable()

        # The radar restarts after sending the ACK.
        self._send_command(
            CMD_RESTART
        )

        # Give the module a moment to reboot.
        time.sleep(0.5)


# ---------------------------------------------------------------------------
# CLI helpers
# ---------------------------------------------------------------------------

def print_parameters(params: DetectionParameters) -> None:
    print("\nDetection Parameters")
    print("--------------------")
    print(f"Maximum distance : {params.max_distance} m")
    print(f"Direction        : {params.direction_name}")
    print(f"Minimum speed    : {params.min_speed} km/h")
    print(f"No-target delay  : {params.delay} s")


def print_sensitivity(sensitivity: Sensitivity) -> None:
    print("\nSensitivity")
    print("-----------")
    print(f"Trigger count    : {sensitivity.trigger_count}")
    print(f"SNR threshold    : {sensitivity.snr_threshold}")
    print(f"Extended 1       : {sensitivity.extended_1}")
    print(f"Extended 2       : {sensitivity.extended_2}")


def main() -> None:

    parser = argparse.ArgumentParser(
        description="Configure the HLK-LD2451 radar."
    )

    parser.add_argument(
        "--port",
        default="/dev/serial0",
        help="Serial port (default: /dev/serial0)",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Current radar baud rate (default: 115200)",
    )

    group = parser.add_mutually_exclusive_group(
        required=True
    )

    group.add_argument(
        "--read",
        action="store_true",
        help="Read current detection parameters.",
    )

    group.add_argument(
        "--read-sensitivity",
        action="store_true",
        help="Read current sensitivity.",
    )

    group.add_argument(
        "--firmware",
        action="store_true",
        help="Read firmware version.",
    )

    group.add_argument(
        "--set-params",
        action="store_true",
        help="Set detection parameters.",
    )

    group.add_argument(
        "--set-sensitivity",
        action="store_true",
        help="Set sensitivity.",
    )

    group.add_argument(
        "--set-baud",
        type=int,
        metavar="BAUD",
        help="Set serial baud rate.",
    )

    group.add_argument(
        "--factory-reset",
        action="store_true",
        help="Restore factory settings.",
    )

    group.add_argument(
        "--restart",
        action="store_true",
        help="Restart the radar.",
    )

    # Detection parameters
    parser.add_argument(
        "--max-distance",
        type=int,
        default=100,
        help="Maximum detection distance in meters.",
    )

    parser.add_argument(
        "--direction",
        choices=["away", "approach", "all"],
        default="all",
        help="Target movement direction.",
    )

    parser.add_argument(
        "--min-speed",
        type=int,
        default=0,
        help="Minimum target speed in km/h.",
    )

    parser.add_argument(
        "--delay",
        type=int,
        default=2,
        help="No-target delay in seconds.",
    )

    # Sensitivity
    parser.add_argument(
        "--trigger-count",
        type=int,
        default=1,
        help="Required consecutive detections.",
    )

    parser.add_argument(
        "--snr-threshold",
        type=int,
        default=4,
        help="SNR threshold.",
    )

    parser.add_argument(
        "--extended-1",
        type=int,
        default=0,
        help="Sensitivity extended parameter 1.",
    )

    parser.add_argument(
        "--extended-2",
        type=int,
        default=0,
        help="Sensitivity extended parameter 2.",
    )

    args = parser.parse_args()

    direction_map = {
        "away": DIRECTION_AWAY,
        "approach": DIRECTION_APPROACH,
        "all": DIRECTION_ALL,
    }

    print(
        f"Opening {args.port} @ {args.baud} baud..."
    )

    with LD2451Configurator(
        port=args.port,
        baudrate=args.baud,
    ) as radar:

        # --------------------------------------------------------------
        # Read parameters
        # --------------------------------------------------------------

        if args.read:

            params = radar.read_parameters()

            print_parameters(params)

        # --------------------------------------------------------------
        # Read sensitivity
        # --------------------------------------------------------------

        elif args.read_sensitivity:

            sensitivity = radar.read_sensitivity()

            print_sensitivity(sensitivity)

        # --------------------------------------------------------------
        # Firmware
        # --------------------------------------------------------------

        elif args.firmware:

            firmware = radar.read_firmware()

            print("\nFirmware")
            print("--------")
            print(f"Radar type     : 0x{firmware.radar_type:04X}")
            print(
                f"Host version   : "
                f"0x{firmware.host_version:04X}"
            )
            print(
                f"Minor version  : "
                f"0x{firmware.minor_version:08X}"
            )

        # --------------------------------------------------------------
        # Set detection parameters
        # --------------------------------------------------------------

        elif args.set_params:

            direction = direction_map[
                args.direction
            ]

            params = radar.set_parameters(
                max_distance=args.max_distance,
                direction=direction,
                min_speed=args.min_speed,
                delay=args.delay,
            )

            print("\nUpdated parameters:")
            print_parameters(params)

        # --------------------------------------------------------------
        # Set sensitivity
        # --------------------------------------------------------------

        elif args.set_sensitivity:

            sensitivity = radar.set_sensitivity(
                trigger_count=args.trigger_count,
                snr_threshold=args.snr_threshold,
                extended_1=args.extended_1,
                extended_2=args.extended_2,
            )

            print("\nUpdated sensitivity:")
            print_sensitivity(sensitivity)

        # --------------------------------------------------------------
        # Baud rate
        # --------------------------------------------------------------

        elif args.set_baud is not None:

            radar.set_baudrate(
                args.set_baud
            )

            print(
                f"\nBaud rate changed to "
                f"{args.set_baud}."
            )

            print(
                "The new baud rate will take effect "
                "after the radar is restarted."
            )

        # --------------------------------------------------------------
        # Factory reset
        # --------------------------------------------------------------

        elif args.factory_reset:

            radar.factory_reset()

            print(
                "\nFactory reset command succeeded."
            )

            print(
                "The factory settings take effect "
                "after the radar is restarted."
            )

        # --------------------------------------------------------------
        # Restart
        # --------------------------------------------------------------

        elif args.restart:

            radar.restart()

            print(
                "\nRadar restart command succeeded."
            )

    print("\nDone.")


if __name__ == "__main__":
    main()