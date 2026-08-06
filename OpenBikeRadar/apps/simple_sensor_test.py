import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ld2451.serial_reader import SerialReader
from ld2451.frame_parser import parse

def main():
    with SerialReader() as reader:
        for frame in reader:
            print(f"Raw frame length={frame.length}  payload={frame.payload.hex(' ')}")

            try:
                radar = parse(frame)
            except ValueError as e:
                print(f"  → parse error: {e}")
                continue

            print(f"Detected {radar.target_count} target(s)")
            for target in radar.targets:
                print(f"  {target}")

if __name__ == "__main__":
    main()