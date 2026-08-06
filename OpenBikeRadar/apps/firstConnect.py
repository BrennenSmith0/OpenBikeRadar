import serial
import threading
import queue
import time
from openant.easy.node import Node
from openant.easy.channel import Channel

#ANT+ Constants
ANT_PUBLIC_NETWORK_KEY = [0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45]
RADAR_DEVICE_TYPE = 40 #0x28 for a bike radar
RADAR_CHANNEL_PERIOD = 8192 #4 hz transmission rate 
RADAR_RF_FREQ = 57 #2457 MHz

#parsing LD2451 constants
HEADER = b'\xf4\xf3\xf2\xf1'
TAIL = b'\xf8\xf7\xf6\xf5'

class RadarStateContainer:
    def __init__(self):
        self._lock = threading.Lock();
        self._targets = [];

    def update_targets(self, new_targets):
        with self._lock:
            self._targets = new_targets

    def get_targets(self):
        with self._lock:
            return list(self._targets)

def ld2451_uart_thread(port="/dev/serial0", 
                       baudrate=115200, state_container=None, stop_event=None):

    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        buffer = bytearray()
        print(f"[+] UART listening on {port} at {baudrate} baud...")
    except serial.SerialException as e:
        print(f"[-] Failed to open serial port {port}: {e}")

    while not stop_event.is_set():
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
            targets, buffer = parse_ld2451_frame(buffer)

            if targets is not None:
                state_container.update_targets(targets)
        else:
            time.sleep(0.01)
    ser.close()
    print("[+] UART thread stopped")


def parse_ld2451_frame(buffer: bytearray):
    """Parses a raw LD2451 byte buffer cleanly without throwing IndexError."""
    HEADER = b'\xf4\xf3\xf2\xf1'
    TAIL = b'\xf8\xf7\xf6\xf5'

    header_idx = buffer.find(HEADER)
    if header_idx == -1:
        return None, buffer[-3:] if len(buffer) >= 3 else buffer

    buffer = buffer[header_idx:]

    # Header(4) + Length(2) + Tail(4) minimum overhead = 10 bytes
    if len(buffer) < 10:
        return None, buffer

    data_len = int.from_bytes(buffer[4:6], byteorder='little')
    total_frame_len = 4 + 2 + data_len + 4

    if len(buffer) < total_frame_len:
        return None, buffer  # Wait for full packet

    frame = buffer[:total_frame_len]
    if frame[-4:] != TAIL:
        return None, buffer[4:]  # Corrupted frame, skip header

    payload = frame[6:6 + data_len]
    
    # Ensure payload contains target count and alarm byte
    if len(payload) < 2:
        return None, buffer[total_frame_len:]

    target_count = payload[0]
    alarm_info = payload[1]

    targets = []
    for i in range(target_count):
        offset = 2 + (i * 5)
        if offset + 5 > len(payload):
            break

        angle_raw = payload[offset]
        angle_deg = angle_raw - 0x80
        distance_m = payload[offset + 1]
        direction = payload[offset + 2]
        speed_kmh = payload[offset + 3]
        snr = payload[offset + 4]

        speed_ms = speed_kmh / 3.6

        if direction == 0x00:  # Approaching
            threat = 2 if speed_kmh > 40 else 1
        else:
            threat = 0

        if threat > 0:
            targets.append({
                'distance': float(distance_m),
                'speed': round(speed_ms, 2),
                'threat': threat,
                'angle': angle_deg,
                'snr': snr
            })

    return targets, buffer[total_frame_len:]


class AntRadarBroadcaster:
    def __init__(self, state_container, device_number=54321):
        self.state_container = state_container
        self.node = Node()
        self.node.set_network_key(0x00, ANT_PUBLIC_NETWORK_KEY)
        
        self.channel = self.node.new_channel(Channel.Type.BIDIRECTIONAL_TRANSMIT)
        self.channel.set_id(device_number, RADAR_DEVICE_TYPE, 1)
        self.channel.set_period(RADAR_CHANNEL_PERIOD)
        self.channel.set_rf_freq(RADAR_RF_FREQ)
        self.message_counter = 0

    def build_page_48(self, targets):
        """Constructs Page 48 (0x30) ANT+ Radar payload."""
        target_count = min(len(targets), 4)
        page_num = 0x30
        byte_1 = target_count & 0x07

        if target_count > 0:
            t1 = targets[0]
            dist_m = min(int(t1.get('distance', 0)), 255)
            speed_ms = min(int(t1.get('speed', 0)), 255)
            threat = t1.get('threat', 1)
        else:
            dist_m = 0
            speed_ms = 0
            threat = 0

        return [
            page_num,
            byte_1,
            dist_m,
            speed_ms,
            threat,
            0xFF, 0xFF, 0xFF  # Padding for targets 2-4
        ]

    def get_next_payload(self, targets):
        self.message_counter += 1
        
        # Every 64th packet (~16 seconds), send Page 80 (Manufacturer Info)
        if self.message_counter % 128 == 64:
            return [0x50, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0x01, 0x00] # Page 80
            
        # Every 128th packet (~32 seconds), send Page 81 (Product Info)
        if self.message_counter % 128 == 0:
            return [0x51, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0x01, 0x00] # Page 81

        # Standard Page 48 (Radar Targets)
        return self.build_page_48(targets)


    def run(self, stop_event):
        self.channel.open()
        print("[+] ANT+ Radar Broadcaster active (4 Hz)...")
        next_tx_time = time.monotonic()

        try:
            while not stop_event.is_set():
                current_targets = self.state_container.get_targets()
                payload = self.get_next_payload(current_targets)
            
                self.channel.send_broadcast_data(payload)

                # Target precise 250ms interval (4 Hz)
                next_tx_time += 0.25
                sleep_duration = next_tx_time - time.monotonic()
            
                if sleep_duration > 0:
                    time.sleep(sleep_duration)
                else:
                    # Reset clock baseline if loop fell behind
                    next_tx_time = time.monotonic()

        finally:
            self.channel.close()
            self.node.stop()
            print("[+] ANT+ thread stopped.")


if __name__ == "__main__":
    state_container = RadarStateContainer()
    stop_event = threading.Event()

    uart_thread = threading.Thread(
        target=ld2451_uart_thread,
        kwargs={
            "port": "/dev/serial0",
            "baudrate": 115200,
            "state_container": state_container,
            "stop_event": stop_event
        },
        daemon=True
    )
    uart_thread.start()

    broadcaster = AntRadarBroadcaster(state_container=state_container, device_number=54321)

    try:
        broadcaster.run(stop_event)
    except KeyboardInterrupt:
        print("\n[!] Shutting down cleanly...")
    finally:
        stop_event.set()
        uart_thread.join(timeout=1.0)