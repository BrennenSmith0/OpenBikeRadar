import serial
import time

port = '/dev/serial0'
header = b'\xf4\xf3\xf2\xf1'
trailer = b'\xf8\xf7\xf6\xf5'
noTarget = b'\x00\x00'

ser = serial.Serial(port, 115200, timeout=1)
print("Serial port opened")

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            if (data[0:4] == header) and (data.endswith(trailer)) and (len(data) > 8):
                payload = data[4:-4]
                
                if payload == noTarget:
                    print("No target")
                else:
                    count = payload[2]
                    alarm = payload[3]

                    for i in range(count):
                        base = 4 + (i * 5)
                        if base + 5 <= len(payload):
                            angle = payload[base] - 0x80
                            dist = payload[base + 1]
                            direction = "Approaching" if payload[base + 2] == 0x01 else "Moving away"
                            speed = payload[base + 3]
                            snr = payload[base + 4]
                            print(f" Object {i+1}: Angle:{angle}deg Dist:{dist}m {direction} Speed:{speed}km/h SNR:{snr}")
        time.sleep(0.05)

except KeyboardInterrupt:
    ser.close()
    print("Closed")