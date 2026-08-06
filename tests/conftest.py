"""
Shared pytest configuration.

The source modules (ld2451/, garmin/, simulator/, tracker/) are plain
folders of scripts, not installable packages -- there's no setup.py or
pyproject.toml, and no __init__.py files. So a test under tests/ld2451/
can't just `import serial_reader`; Python has no way to find it.

This file adds each source folder to sys.path once, at collection time,
so any test anywhere under tests/ can import the module it's testing by
its plain filename, e.g.:

    from serial_reader import SerialReader   # ld2451/serial_reader.py
    from tracker import Tracker              # tracker/tracker.py

As new module folders gain tests, add them to SOURCE_DIRS below.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

SOURCE_DIRS = [
    "ld2451",
    "garmin",
    "simulator",
    "tracker",
]

for _name in SOURCE_DIRS:
    _path = str(REPO_ROOT / _name)
    if _path not in sys.path:
        sys.path.insert(0, _path)