"""
serial_reader.py
Handles reading in the bytes from the HLK-LD2451 sensor.
This module is responsible ONLY for reading complete frames from the
serial port. It does not interpret or decode the payload.

Frame format:
-Header: 4 bytes
-Length: 2 bytes
-Payload: N bytes
-Trailer: 4 bytes

For the LD2451 radar on signal frames:

Header  = F4 F3 F2 F1
Trailer = F8 F7 F6 F5
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, Optional
import struct
import time
import config 

import serial


@dataclass(frozen=True)
class Frame:
    """
    One complete frame received from the serial port.
    """

    header: bytes
    length: int
    payload: bytes
    trailer: bytes

    def to_bytes(self) -> bytes:
        """Reconstruct the original frame."""
        return (
            self.header
            + struct.pack("<H", self.length)
            + self.payload
            + self.trailer
        )

    def hex(self, separator: str = " ") -> str:
        """Return frame as a hexadecimal string."""
        return self.to_bytes().hex(separator)

    def __len__(self) -> int:
        return self.length

    def __repr__(self):
        return (
            f"Frame(length={self.length}, "
            f"payload={self.payload.hex(' ')})"
        )

    def __post_init__(self):
        if self.length != len(self.payload):
            raise ValueError(
                "Frame length does not match payload length"
        )


class SerialReader:
    """
    Reads complete framed packets from a serial port.

    This class knows nothing about the payload contents.
    It simply returns validated Frame objects.
    """

    DEFAULT_HEADER = b"\xF4\xF3\xF2\xF1"
    DEFAULT_TRAILER = b"\xF8\xF7\xF6\xF5"

    def __init__(
        self,
        port: str = config.SERIAL_PORT,
        baudrate: int = config.BAUDRATE,
        timeout: float = 0.1,
        header: bytes = DEFAULT_HEADER,
        trailer: bytes = DEFAULT_TRAILER,
        serial_port: Optional[serial.Serial] = None,
    ):

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.header = header
        self.trailer = trailer

        self.ser = serial_port
        self._owns_serial = serial_port is None
        self.buffer = bytearray()

    def open(self) -> None:
        """Open the serial port."""
        if self.ser is None:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
            )

        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        if (
            self._owns_serial
            and self.ser
            and self.ser.is_open
        ):
            self.ser.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __iter__(self) -> Iterator[Frame]:
        """
        Iterate forever, yielding complete frames.
        """

        while True:

            frame = self.read_frame()

            if frame is not None:
                yield frame

    
    def read_frame(self) -> Optional[Frame]:
        """
        Read one complete frame.

        Returns
        -------
        Frame
            A validated frame.

        None
            If a complete frame has not yet arrived.
        """

        if self.ser is None:
            raise RuntimeError("Serial port has not been opened.")

        if self.ser.in_waiting:
            self.buffer.extend(self.ser.read(self.ser.in_waiting))

        while True:

            start = self.buffer.find(self.header)

            if start == -1:

                # No header found.
                # Keep only enough bytes to detect a split header.
                if len(self.buffer) > len(self.header):
                    del self.buffer[:-len(self.header)]

                return None

            # Remove junk before header.
            if start:
                del self.buffer[:start]

            # Need header + length.
            if len(self.buffer) < 6:
                return None

            payload_length = struct.unpack_from("<H", self.buffer, 4)[0]

            total_length = (
                len(self.header)
                + 2
                + payload_length
                + len(self.trailer)
            )

            if len(self.buffer) < total_length:
                return None

            trailer_start = total_length - len(self.trailer)

            if (
                self.buffer[trailer_start:total_length]
                != self.trailer
            ):
                # Corrupt frame.
                # Advance one byte and search again.
                del self.buffer[0]
                continue

            payload = bytes(
                self.buffer[
                    len(self.header) + 2 : trailer_start
                ]
            )

            frame = Frame(
                header=self.header,
                length=payload_length,
                payload=payload,
                trailer=self.trailer,
            )

            del self.buffer[:total_length]

            return frame