
from ld2451.serial_reader import SerialReader
from ld2451.frame_parser import parse

def main():
    with SerialReader() as reader:
        for frame in reader:
            print(f"Raw frame length={frame.length}   header={frame.header.hex(' ')}  payload={frame.payload.hex(' ')}, trailer={frame.trailer.hex(' ')}")

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
