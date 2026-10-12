#!/usr/bin/env python3
"""
tools/grab_frame.py

Captures hex-encoded PPM frames and benchmark logs streamed over UART
from the ESP32-S3 running the ESR demo.
Usage:
    python3 tools/grab_frame.py /dev/ttyACM0
    python3 tools/grab_frame.py path/to/saved_log.txt
"""

import os
import sys

if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <serial_port_or_logfile>")
    sys.exit(1)

src = sys.argv[1]
live = not os.path.isfile(src)
if live:
    import serial
    stream = serial.Serial(src, 115200, timeout=5)
else:
    stream = open(src, "r", errors="ignore")


def read_line():
    raw = stream.readline()
    if isinstance(raw, bytes):
        raw = raw.decode(errors="ignore")
    return raw


log, data = [], []
w = h = 0
capturing = False
started = False

print("Waiting for frame report from ESP32-S3..." if live else "Replaying log...")

while True:
    line = read_line()
    if line == "":
        if live:
            continue
        break
    line = line.rstrip("\r\n")

    if not started:
        if line.startswith("===") or line.startswith("---") or line.startswith("PPM_BEGIN"):
            started = True
            print(line)
            log.append(line)
            if not line.startswith("PPM_BEGIN"):
                continue
        else:
            continue

    if line.startswith("PPM_BEGIN"):
        _, w, h = line.split()
        w, h = int(w), int(h)
        capturing, data = True, []
        print(f"[receiving {w}x{h} frame, reading UART hex stream...]")
    elif line.startswith("PPM_END") and capturing:
        capturing = False
        print("[frame received]")
    elif capturing:
        data.append(line)
    else:
        print(line)
        log.append(line)
        if line.startswith("Done"):
            break

with open("bringup_log.txt", "w") as f:
    f.write("\n".join(log) + "\n")
print("Saved bringup_log.txt")

if data:
    raw = bytes.fromhex("".join(data))
    assert len(raw) == w * h * 3, f"expected {w*h*3} bytes, got {len(raw)}"
    with open("frame.ppm", "wb") as f:
        f.write(f"P6\n{w} {h}\n255\n".encode())
        f.write(raw)
    print("Saved frame.ppm")
    try:
        from PIL import Image
        Image.frombytes("RGB", (w, h), raw).resize((w * 4, h * 4), Image.NEAREST).save("frame.png")
        print("Saved frame.png (4x enlarged)")
    except ImportError:
        print("Install Pillow (pip install pillow) to also generate frame.png")
