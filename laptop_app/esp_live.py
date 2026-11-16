"""SETUP DIAGNOSTIC ONLY: live (~2-4 fps) camera preview for aiming the camera.

Repeatedly requests one grayscale frame with the 'G' setup command and shows it with the ROI box.
The study app never requests images; all recognition runs on the ESP32.

  python esp_live.py --port COM4        (ESC / close window to quit)
"""
import argparse
import re
import time

import numpy as np
import pygame

from esp_console import open_port


def grab(ser):
    ser.reset_input_buffer()
    ser.write(b"G\n")
    buf, t0, last = b"", time.time(), time.time()
    while time.time() - t0 < 3:
        chunk = ser.read(max(1, ser.in_waiting))   # bulk read (readline() is slow on Windows)
        if chunk:
            buf += chunk
            last = time.time()
            if buf.count(b"\n#I") >= 121 and buf.endswith(b"\n"):
                break
        elif b"#IMG" in buf and time.time() - last > 0.15:
            break                       # dump finished (a dropped row or two is padded below)
    rows, started = [], False
    for line in buf.decode(errors="replace").splitlines():
        line = line.strip()
        if line.startswith("#IMG"):
            started, rows = True, []
        elif started and line.startswith("#I") and len(line) == 322:
            rows.append(bytes.fromhex(line[2:]))
    if len(rows) < 100:
        return None
    rows += [rows[-1]] * (120 - len(rows))
    return np.frombuffer(b"".join(rows), dtype=np.uint8).reshape(120, 160)


def read_roi(ser):
    ser.reset_input_buffer()
    ser.write(b"p\n")
    t0 = time.time()
    while time.time() - t0 < 1.5:
        line = ser.readline().decode(errors="replace")
        m = re.search(r"roi=(\d+),(\d+),(\d+),(\d+)", line)
        if m:
            return [int(v) for v in m.groups()]
        if line.startswith("#P"):
            break
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--scale", type=int, default=5)
    ap.add_argument("--roi", default="", help="x0,y0,x1,y1 in %% to draw the analysis region, e.g. 0,0,100,62")
    a = ap.parse_args()
    ser = open_port(a.port)
    ser.write(b"v0\n")
    roi = [int(v) for v in a.roi.split(",")] if a.roi else read_roi(ser)
    ser.timeout = 0.05
    pygame.init()
    S = a.scale
    win = pygame.display.set_mode((160 * S, 120 * S + 40))
    pygame.display.set_caption("ESP32 camera live preview (setup only)")
    font = pygame.font.SysFont("Microsoft JhengHei,Arial", 20)
    run, n, t0 = True, 0, time.time()
    show_dark = True
    while run:
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                run = False
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_h:
                show_dark = not show_dark
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_e:
                ser.write(b"e\n")          # re-run auto-exposure (2 s) + background (keep hand out)
                time.sleep(4.0)
        img = grab(ser)
        if img is None:
            continue
        n += 1
        rgb = np.repeat(img[:, :, None], 3, axis=2).astype(np.int16)
        if show_dark:   # approx. what the tracker segments: pixels clearly darker than the typical background
            dark = img.astype(np.int16) < (np.median(img) - 22)
            rgb[dark] = (rgb[dark] * 0.4 + np.array([255, 0, 0]) * 0.6).astype(np.int16)
        rgb = rgb.clip(0, 255).astype(np.uint8).transpose(1, 0, 2).copy()
        surf = pygame.transform.scale(pygame.surfarray.make_surface(rgb), (160 * S, 120 * S))
        win.fill((20, 20, 20))
        win.blit(surf, (0, 0))
        # centre cross
        cx, cy = 80 * S, 60 * S
        pygame.draw.line(win, (0, 200, 0), (cx - 20, cy), (cx + 20, cy), 2)
        pygame.draw.line(win, (0, 200, 0), (cx, cy - 20), (cx, cy + 20), 2)
        if roi:
            x0, y0, x1, y1 = roi
            pygame.draw.rect(win, (255, 60, 60), (x0 * 160 * S // 100, y0 * 120 * S // 100,
                             (x1 - x0) * 160 * S // 100, (y1 - y0) * 120 * S // 100), 2)
        fps = n / max(time.time() - t0, 1e-3)
        txt = f"亮度={img.mean():.0f}  {fps:.1f} fps   紅色=系統會當成手的區域(只有手該是紅的)  H=開關紅色  E=重調亮度(手移開)  ESC離開"
        win.blit(font.render(txt, True, (230, 230, 230)), (8, 120 * S + 8))
        pygame.display.flip()
    ser.close()
    pygame.quit()


if __name__ == "__main__":
    main()
