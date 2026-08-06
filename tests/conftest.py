"""
Shared pytest configuration.

ld2451/ is now a real package (it has __init__.py). garmin/, simulator/,
and tracker/ are still plain folders of scripts with no __init__.py.

Rather than mixing two import styles, everything is imported package-
style off the repo root, e.g.:

    from ld2451.serial_reader import SerialReader
    from ld2451.frame_parser import parse

So this file adds only the repo root to sys.path, once, at collection
time. Adding module folders directly (the old approach) is what caused
the `ModuleNotFoundError: No module named 'ld2451.serial_reader'` bug:
pytest also puts tests/ on sys.path to load this file, and since
tests/ld2451/ has no __init__.py, Python was treating it as an implicit
namespace package literally named "ld2451" -- shadowing the real one.

If garmin/, simulator/, or tracker/ later get their own __init__.py
files (recommended once they have real code), tests for them can use
the same `from garmin.varia import ...` style with no further setup.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))