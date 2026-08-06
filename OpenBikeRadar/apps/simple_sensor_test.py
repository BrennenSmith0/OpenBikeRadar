from ld2451.serial_reader import SerialReader
from ld2451.frame_parser import parse

def main():
    with SerialReader() as reader:
        for frame in reader:
            radar = parse(frame)

            print(f"\nDetected {radar.target_count} target(s)")

            for target in radar.targets:
                print(f"  {target}")

if __name__ == "__main__":
    main()