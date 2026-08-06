#!/usr/bin/env python3
"""
configure_sensor.py

One-time configuration utility for the HLK-LD2451 24GHz radar module.

Default behavior sets:
  - Movement direction : 0x02 (detect BOTH approaching and departing targets)
  - Minimum speed       : 0 km/h (no minimum threshold -- report all speeds)

These parameters are written to the LD2451's non-volatile memory, so this
only needs to be run once (or whenever you want to change the config) --
the radar keeps the settings across power cycles. It does NOT need to be
run every time main.py starts.

Protocol reference: HiLink LD2451 configuration protocol.
Command frames use header FD FC FB FA / tail 04 03 02 01 -- this is
DIFFERENT from the F4F3F2F1 / F8F7F6F5 data-report frames main.py parses.

Usage:
    python configure_sensor.py                       # direction=all, min-speed=0, defaults otherwise
    python configure_sensor.py --max-distance 80 --delay 3
    python configure_sensor.py --read-only            # just print current config, change nothing
"""

import argparse
import struct
import sys
import time

import serial

FRAME_HEADER = bytes([0xFD, 0xFC, 0xFB, 0xFA])
FRAME_TAIL = bytes([0x04, 0x03, 0x02, 0x01])

CMD_ENABLE_CONFIG = 0x00FF
CMD_END_CONFIG = 0x00FE
CMD_SET_DETECTION_PARAMS = 0x0002
CMD_READ_DETECTION_PARAMS = 0x0012

DIRECTION_BYTES = {"away": 0x00, "approach": 0x01, "all": 0x02}
DIRECTION_NAMES = {v: k for k, v in DIRECTION_BYTES.items()}


def build_frame(cmd_word: int, cmd_value: bytes = b"") -> bytes:
    data = struct.pack("<H", cmd_word) + cmd_value
    length = struct.pack("<H", len(data))
    return FRAME_HEADER + length + data + FRAME_TAIL


def read_frame(ser: serial.Serial, timeout: float = 2.0):
    """Read one command-ACK frame (FD FC FB FA ... 04 03 02 01)."""
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            start = buf.find(FRAME_HEADER)
            if start != -1 and len(buf) >= start + 6:
                length = struct.unpack("<H", buf[start + 4:start + 6])[0]
                end = start + 6 + length + 4  # + tail
                if len(buf) >= end:
                    frame = buf[start:end]
                    if frame.endswith(FRAME_TAIL):
                        return frame
                    buf = buf[start + 4:]  # bad frame, keep scanning
        else:
            time.sleep(0.01)
    return None


def send_command(ser: serial.Serial, cmd_word: int, cmd_value: bytes = b"", label: str = "") -> bytes:
    frame = build_frame(cmd_word, cmd_value)
    ser.write(frame)
    print(f"  -> {label or hex(cmd_word)}: {frame.hex(' ').upper()}")

    resp = read_frame(ser)
    if resp is None:
        raise RuntimeError(f"No ACK received for {label or hex(cmd_word)} (timed out)")
    print(f"  <- {label or hex(cmd_word)}: {resp.hex(' ').upper()}")

    length = struct.unpack("<H", resp[4:6])[0]
    value = resp[6:6 + length]
    status = struct.unpack("<H", value[2:4])[0]
    if status != 0:
        raise RuntimeError(f"{label or hex(cmd_word)} failed (status={status})")
    return value[4:]  # payload after the 2-byte ack-cmd-word + 2-byte status


def describe(params: bytes) -> str:
    max_dist, direction, min_speed, delay = params[0], params[1], params[2], params[3]
    dir_name = DIRECTION_NAMES.get(direction, f"unknown(0x{direction:02X})")
    return f"max_dist={max_dist}m  direction={dir_name}  min_speed={min_speed}km/h  no_target_delay={delay}s"


def main():
    ap = argparse.ArgumentParser(description="Configure HLK-LD2451 target detection parameters")
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--max-distance", type=int, default=100, help="Max detection distance in meters (10-100)")
    ap.add_argument("--direction", choices=["away", "approach", "all"], default="all",
                     help="Which movement directions to report")
    ap.add_argument("--min-speed", type=int, default=0, help="Minimum speed threshold in km/h (0-120)")
    ap.add_argument("--delay", type=int, default=2, help="No-target delay time in seconds (0-10)")
    ap.add_argument("--read-only", action="store_true", help="Only read back current config; make no changes")
    args = ap.parse_args()

    if not (10 <= args.max_distance <= 100):
        sys.exit("error: --max-distance must be 10-100")
    if not (0 <= args.min_speed <= 120):
        sys.exit("error: --min-speed must be 0-120")
    if not (0 <= args.delay <= 10):
        sys.exit("error: --delay must be 0-10")

    direction_byte = DIRECTION_BYTES[args.direction]

    print(f"Opening {args.port} @ {args.baud} baud...")
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        time.sleep(0.1)
        ser.reset_input_buffer()

        print("\n[1/4] Enabling configuration mode...")
        send_command(ser, CMD_ENABLE_CONFIG, struct.pack("<H", 0x0001), "enable-config")

        print("\n[2/4] Reading current detection parameters...")
        current = send_command(ser, CMD_READ_DETECTION_PARAMS, b"", "read-params")
        print(f"  current: {describe(current)}")

        if not args.read_only:
            new_value = bytes([args.max_distance, direction_byte, args.min_speed, args.delay])
            print(f"\n[3/4] Writing new detection parameters: {describe(new_value)}")
            send_command(ser, CMD_SET_DETECTION_PARAMS, new_value, "set-params")

            verify = send_command(ser, CMD_READ_DETECTION_PARAMS, b"", "verify-params")
            if bytes(verify[:4]) == new_value:
                print(f"  verified: {describe(verify)}")
            else:
                print(f"  WARNING: readback ({describe(verify)}) does not match what was requested")
        else:
            print("\n[3/4] --read-only set, skipping write")

        print("\n[4/4] Ending configuration mode...")
        send_command(ser, CMD_END_CONFIG, b"", "end-config")

    print("\nDone. Settings are stored in the LD2451's non-volatile memory and "
          "persist across power cycles -- you don't need to re-run this at every boot.")


if __name__ == "__main__":
    main()