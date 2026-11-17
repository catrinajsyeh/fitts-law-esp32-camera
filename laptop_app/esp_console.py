"""Tiny serial console for setting up the ESP32 hand tracker.

Usage:
  python esp_console.py --port COM4                 # interactive: type commands (b, c, m, o0, v, d, p, s ...)
  python esp_console.py --port COM4 --send "p" --seconds 3   # send commands, print output for N seconds
"""
import argparse
import sys
import threading
import time

import serial


def open_port(port):
    """Open the ESP32-S3 USB-CDC port WITHOUT toggling DTR/RTS (which would reset the board)."""
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.1
    if sys.platform == "win32":
        s.dtr = False
        s.rts = False
    s.open()
    return s


def reader(ser, show_moves, stop):
    while not stop.is_set():
        try:
            line = ser.readline().decode(errors="replace").rstrip()
        except Exception as e:
            print(f"[read error] {type(e).__name__}: {e}", flush=True)
            time.sleep(0.3)
            continue
        if not line:
            continue
        if line.startswith("M,") and not show_moves:
            continue
        print(line, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--send", default="", help="commands separated by ';' e.g. 'v;d'")
    ap.add_argument("--seconds", type=float, default=0, help="non-interactive: listen this long then exit")
    ap.add_argument("--moves", action="store_true", help="also print M (cursor) lines")
    a = ap.parse_args()

    ser = open_port(a.port)
    stop = threading.Event()
    t = threading.Thread(target=reader, args=(ser, a.moves, stop), daemon=True)
    t.start()
    for c in [c for c in a.send.split(";") if c]:
        ser.write((c + "\n").encode())
        time.sleep(0.15)
    if a.seconds > 0:
        time.sleep(a.seconds)
    else:
        try:
            for line in sys.stdin:
                ser.write((line.strip() + "\n").encode())
        except KeyboardInterrupt:
            pass
    stop.set()
    t.join(timeout=1)
    ser.close()


if __name__ == "__main__":
    main()
