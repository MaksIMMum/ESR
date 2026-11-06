#!/usr/bin/env python3
"""
tools/image2header.py

Converts an image (JPG, PNG, PPM, BMP) into a C header containing an RGB565 
array ready to be compiled directly into ESP32 Flash memory.

Usage:
    python3 tools/image2header.py my_photo.jpg -o main/test_photo.h
    python3 tools/image2header.py --generate-sample -o main/test_photo.h
"""

import sys
import os
import math
import argparse

def rgb_to_rgb565(r, g, b):
    """Convert RGB888 (0-255) to RGB565 big-endian integer for SPI."""
    r5 = (r >> 3) & 0x1F
    g6 = (g >> 2) & 0x3F
    b5 = (b >> 3) & 0x1F
    val = (r5 << 11) | (g6 << 5) | b5
    # Byte-swap for big-endian SPI transmission
    return ((val >> 8) & 0xFF) | ((val & 0xFF) << 8)

def generate_sample_photo(width=240, height=320):
    """Generates a rich sunset landscape scene using pure Python math."""
    pixels = []
    sun_cx, sun_cy = width * 0.5, height * 0.42
    sun_radius = width * 0.22

    for y in range(height):
        # Sky gradient: deep twilight blue -> warm orange -> bright yellow
        sky_t = y / (height * 0.7)
        if sky_t < 0.5:
            t = sky_t / 0.5
            sky_r = int(20 + t * (235 - 20))
            sky_g = int(30 + t * (120 - 30))
            sky_b = int(80 + t * (50 - 80))
        else:
            t = (sky_t - 0.5) / 0.5
            sky_r = int(235 + t * (255 - 235))
            sky_g = int(120 + t * (200 - 120))
            sky_b = int(50 + t * (70 - 50))

        # Mountain silhouette height
        mtn_h1 = int(height * 0.65 + math.sin(0.04 * (y * 0 + x if False else 0)) * 0) # base

        for x in range(width):
            # Mountain layers using sine waves
            m1 = int(height * 0.68 + 22.0 * math.sin(x * 0.035) + 12.0 * math.cos(x * 0.08))
            m2 = int(height * 0.78 + 15.0 * math.sin(x * 0.05 + 1.2) + 8.0 * math.sin(x * 0.12))

            dist_sun = math.hypot(x - sun_cx, y - sun_cy)

            if y >= m2:
                # Foreground dark mountains
                r, g, b = 15, 20, 25
            elif y >= m1:
                # Midground purple mountain ridge
                fade = (y - m1) / max(1, (m2 - m1))
                r = int(60 + fade * 20)
                g = int(35 + fade * 15)
                b = int(75 + fade * 25)
            elif dist_sun < sun_radius:
                # Bright glowing sun
                glow = 1.0 - (dist_sun / sun_radius)
                r = 255
                g = int(220 + glow * 35)
                b = int(140 + glow * 100)
            elif dist_sun < sun_radius * 1.6:
                # Sun atmosphere glow halo
                glow = 1.0 - ((dist_sun - sun_radius) / (sun_radius * 0.6))
                r = min(255, int(sky_r * (1 - glow * 0.6) + 255 * glow * 0.6))
                g = min(255, int(sky_g * (1 - glow * 0.5) + 220 * glow * 0.5))
                b = min(255, int(sky_b * (1 - glow * 0.4) + 120 * glow * 0.4))
            else:
                # Ambient sky
                r, g, b = sky_r, sky_g, sky_b

            pixels.append((r, g, b))

    return width, height, pixels

def load_ppm(path):
    """Loads a binary P6 PPM file using stdlib."""
    with open(path, "rb") as f:
        header = []
        while len(header) < 3:
            line = f.readline().decode().strip()
            if line and not line.startswith("#"):
                header.extend(line.split())
        magic, w, h = header[0], int(header[1]), int(header[2])
        _ = f.readline() # 255
        raw = f.read(w * h * 3)
        pixels = []
        for i in range(0, len(raw), 3):
            pixels.append((raw[i], raw[i+1], raw[i+2]))
        return w, h, pixels

def main():
    parser = argparse.ArgumentParser(description="Convert an image to C header (RGB565)")
    parser.add_argument("input", nargs="?", help="Input image file (JPG, PNG, PPM, BMP)")
    parser.add_argument("-o", "--output", default="main/test_photo.h", help="Output C header file")
    parser.add_argument("--generate-sample", action="store_true", help="Generate a sample 240x320 sunset landscape")
    parser.add_argument("--width", type=int, default=240, help="Target display width")
    parser.add_argument("--height", type=int, default=320, help="Target display height")
    args = parser.parse_args()

    if args.generate_sample or not args.input:
        print(f"Generating procedural sample photo ({args.width}x{args.height})...")
        w, h, pixels = generate_sample_photo(args.width, args.height)
    elif args.input.lower().endswith(".ppm"):
        w, h, pixels = load_ppm(args.input)
    else:
        try:
            from PIL import Image
            img = Image.open(args.input).convert("RGB")
            if img.size != (args.width, args.height):
                print(f"Resizing {args.input} from {img.size} to ({args.width}, {args.height})...")
                img = img.resize((args.width, args.height), Image.Resampling.LANCZOS)
            w, h = img.size
            pixels = list(img.getdata())
        except ImportError:
            print("Pillow is not installed. To convert JPG/PNG, run: pip install pillow")
            print("Falling back to generating procedural sample photo...")
            w, h, pixels = generate_sample_photo(args.width, args.height)

    print(f"Writing {w}x{h} ({len(pixels)} pixels) into '{args.output}'...")
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    with open(args.output, "w") as f:
        f.write("#pragma once\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"#define PHOTO_WIDTH  {w}\n")
        f.write(f"#define PHOTO_HEIGHT {h}\n\n")
        f.write("// Stored in ESP32 Flash memory (RODATA)\n")
        f.write("static const uint16_t photo_data[] = {\n")

        line = []
        for i, (r, g, b) in enumerate(pixels):
            rgb565 = rgb_to_rgb565(r, g, b)
            line.append(f"0x{rgb565:04X}")
            if len(line) >= 12:
                f.write("    " + ", ".join(line) + ",\n")
                line = []
        if line:
            f.write("    " + ", ".join(line) + "\n")
        f.write("};\n")

    print(f"Done! Created '{args.output}' ({os.path.getsize(args.output) // 1024} KB).")

if __name__ == "__main__":
    main()
