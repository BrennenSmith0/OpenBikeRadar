"""
test_serial_reader.py

Unit tests for ld2451/serial_reader.py.

SerialReader.read_frame() is driven by a fake serial port so these tests
run with no hardware attached. Each test feeds raw bytes through
FakeSerial and checks what SerialReader hands back.

Covers:
    - reads one frame
    - reads split frame
    - reads two frames
    - skips garbage
    - rejects bad trailer
    - keeps partial header
"""

import struct

import pytest

from ld2451.serial_reader import SerialReader  # path set up by tests/conftest.py

HEADER = SerialReader.DEFAULT_HEADER
TRAILER = SerialReader.DEFAULT_TRAILER


def build_frame(payload: bytes, header: bytes = HEADER, trailer: bytes = TRAILER) -> bytes:
    """Build one raw on-wire frame for the given payload."""
    return header + struct.pack("<H", len(payload)) + payload + trailer


class FakeSerial:
    """
    Minimal stand-in for serial.Serial.

    Bytes given to feed() sit in a queue and are only handed back once,
    the same way in_waiting/read behave on a real port: SerialReader
    only sees bytes that arrived since its last read.
    """

    def __init__(self):
        self._queue = bytearray()

    def feed(self, data: bytes) -> None:
        self._queue.extend(data)

    @property
    def in_waiting(self) -> int:
        return len(self._queue)

    def read(self, n: int) -> bytes:
        chunk = bytes(self._queue[:n])
        del self._queue[:n]
        return chunk


@pytest.fixture
def fake_serial():
    return FakeSerial()


@pytest.fixture
def reader(fake_serial):
    return SerialReader(serial_port=fake_serial)


def test_reads_one_frame(reader, fake_serial):
    payload = b"\x00\x0a\x01\x2c\x10"
    fake_serial.feed(build_frame(payload))

    frame = reader.read_frame()

    assert frame is not None
    assert frame.header == HEADER
    assert frame.trailer == TRAILER
    assert frame.length == len(payload)
    assert frame.payload == payload
    assert bytes(reader.buffer) == b""  # frame fully consumed, nothing left buffered


def test_reads_split_frame(reader, fake_serial):
    payload = b"\x01\x14\x01\x1e\x08"
    raw = build_frame(payload)

    # First chunk: header + length + one payload byte. Not a full frame yet.
    split_point = 7
    fake_serial.feed(raw[:split_point])
    assert reader.read_frame() is None

    # Second chunk: the rest of the payload + trailer arrives.
    fake_serial.feed(raw[split_point:])
    frame = reader.read_frame()

    assert frame is not None
    assert frame.payload == payload


def test_reads_two_frames(reader, fake_serial):
    payload_a = b"\x00\x0a\x01\x2c\x10"
    payload_b = b"\x01\x14\x01\x1e\x08"
    fake_serial.feed(build_frame(payload_a) + build_frame(payload_b))

    first = reader.read_frame()
    second = reader.read_frame()

    assert first is not None and first.payload == payload_a
    assert second is not None and second.payload == payload_b


def test_skips_garbage(reader, fake_serial):
    payload = b"\x00\x0a\x01\x2c\x10"
    garbage = b"\x11\x22\x33\xff\x00"
    fake_serial.feed(garbage + build_frame(payload))

    frame = reader.read_frame()

    assert frame is not None
    assert frame.payload == payload


def test_rejects_bad_trailer(reader, fake_serial):
    payload = b"\x00\x0a\x01\x2c\x10"
    raw = bytearray(build_frame(payload))
    raw[-1] ^= 0xFF  # corrupt the last trailer byte
    fake_serial.feed(bytes(raw))

    # Corrupt frame must never be handed back as valid data.
    assert reader.read_frame() is None

    # Reader should recover and pick up the next good frame.
    fake_serial.feed(build_frame(payload))
    frame = reader.read_frame()

    assert frame is not None
    assert frame.payload == payload


def test_keeps_partial_header(reader, fake_serial):
    payload = b"\x00\x0a\x01\x2c\x10"
    raw = build_frame(payload)

    # Only the first 2 bytes of the 4-byte header arrive.
    fake_serial.feed(raw[:2])
    assert reader.read_frame() is None
    assert bytes(reader.buffer) == raw[:2]  # partial header held, not dropped

    # Rest of the header, plus the whole frame body, arrives next.
    fake_serial.feed(raw[2:])
    frame = reader.read_frame()

    assert frame is not None
    assert frame.payload == payload