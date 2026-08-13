from time import monotonic

from ld2451.serial_reader import SerialReader
from ld2451.frame_parser import parse
from tracker.tracker import Tracker


def main():

    tracker = Tracker()

    last_time = monotonic()

    with SerialReader() as reader:

        for frame in reader:

            now = monotonic()
            dt = now - last_time
            last_time = now

            radar = parse(frame)

            tracked = tracker.update(radar.targets, dt)

            print("=" * 60)
            print("Radar Targets")

            for target in radar.targets:
                print(f"  {target}")

            print("\nTracked Targets")

            for target in tracked:
                print(f"  {target}")


if __name__ == "__main__":
    main()