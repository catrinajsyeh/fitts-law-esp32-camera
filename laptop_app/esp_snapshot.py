"""SETUP DIAGNOSTIC ONLY: grab one 160x120 grayscale frame from the ESP32 ('G' command) and save a PNG.

Used to check camera placement / lighting before the study. The study app never requests images;
all recognition runs on the ESP32.

  python esp_snapshot.py --port COM4 --out snap.png
"""
import argparse
import time

import numpy as np
import pygame

from esp_console import open_port


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--out", default="snap.png")
    ap.add_argument("--scale", type=int, default=4)
    a = ap.parse_args()
    ser = open_port(a.port)
    ser.reset_input_buffer()
    ser.write(b"G\n")
    rows, t0, started = [], time.time(), False
    while time.time() - t0 < 5 and len(rows) < 120:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("#IMG"):
            started = True
        elif started and line.startswith("#I"):
            h = line[2:]
            if len(h) == 320:
                rows.append(bytes.fromhex(h))
    ser.close()
    if len(rows) < 110:
        raise SystemExit(f"incomplete frame ({len(rows)} rows)")
    rows += [rows[-1]] * (120 - len(rows))  # pad the odd dropped line
    img = np.frombuffer(b"".join(rows), dtype=np.uint8).reshape(120, 160)
    rgb = np.repeat(img[:, :, None], 3, axis=2).transpose(1, 0, 2).copy()
    surf = pygame.surfarray.make_surface(rgb)
    surf = pygame.transform.scale(surf, (160 * a.scale, 120 * a.scale))
    pygame.image.save(surf, a.out)
    print(f"saved {a.out}  min={img.min()} max={img.max()} mean={img.mean():.1f}")


if __name__ == "__main__":
    main()
