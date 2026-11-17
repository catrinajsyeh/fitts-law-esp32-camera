"""1-D Fitts' Law drag-and-drop study driven by the ESP32 camera hand tracker.

The ESP32 does all vision; this app only receives text events over USB serial:
  M,<t_ms>,<x>,<y>   cursor position (0..10000)    B,<t_ms>,<1|0>,<g|l>  button
  T,<t_ms>,<1|0>     tracking state                #...                  status text

Run:
  python fitts_app.py --pid P01 --port COM4
  python fitts_app.py --pid TEST --mouse      # app debugging with the real mouse (never for recorded data)

Phases: SETUP -> PRACTICE -> RECORDED (9 blocks x 10 successful trials) -> DONE
Keys (setup screen): ENTER start practice | B background | C calibrate range | M mirror |
                     0-3 arm-entry edge | S save ESP params | ESC quit
Keys (any time):     ESC quit (data is written after every attempt)  |  SPACE continue after a break
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import queue
import random
import sys
import threading
import time

if sys.platform == "win32":  # use physical pixels so D and W are true display pixels
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except Exception:
        pass

import numpy as np
import pygame

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(os.path.dirname(HERE), "data")

# ---------------- study configuration (keep constant for all participants) ----------------
WIDTHS = [120, 240, 480]         # px, object = target width (W=80 was below the camera's precision: +-16 px)
DISTANCES = [480, 1080, 1920]    # px, object start centre -> target centre  (8 distinct IDs, 1.00 - 4.09 bits)
BAR_HEIGHT = 420                 # px, object = target height
# A 2440 px display keeps these sizes. A narrower display shrinks distances (widths stay if they fit).


def fit_task_to_screen(screen_w, screen_h):
    """Keep the preferred sizes when they fit. Otherwise shrink so D and W stay on this display."""
    global WIDTHS, DISTANCES, BAR_HEIGHT, PRACTICE_CONDITION
    margin = 80
    widths = list(WIDTHS)
    distances = list(DISTANCES)
    scaled = False
    room = screen_w - margin - max(widths)
    if room < max(distances):
        scaled = True
        if room >= 360:
            picked = []
            for value in (max(100, int(room * 0.30)), max(160, int(room * 0.60)), int(room)):
                if value not in picked:
                    picked.append(value)
            while len(picked) < 3:
                picked.append(picked[-1] + 50)
            distances = sorted(picked)[:3]
        else:
            scale = max(0.35, (screen_w - margin) / (max(DISTANCES) + max(WIDTHS)))
            widths = [max(80, int(round(w * scale))) for w in WIDTHS]
            distances = [max(100, int(round(d * scale))) for d in DISTANCES]
    height_cap = max(160, screen_h - 240)
    if BAR_HEIGHT > height_cap:
        BAR_HEIGHT = height_cap
        scaled = True
    WIDTHS = widths
    DISTANCES = distances
    PRACTICE_CONDITION = (DISTANCES[1], WIDTHS[1])
    return scaled
SUCCESS_PER_CONDITION = 10
OVERLAP_CRITERION = 0.80         # released object must overlap target by > 80 % of its area
PRACTICE_ATTEMPTS = 5            # practice block: one medium condition repeated, not analysed
PRACTICE_CONDITION = (1080, 240)  # (D, W) used for every practice attempt (ID 2.46 bits)
FEEDBACK_MS = 700

COL = dict(bg=(28, 30, 36), text=(225, 228, 235), dim=(130, 135, 150), target=(70, 110, 200),
           target_edge=(140, 175, 255), obj=(235, 170, 60), obj_drag=(255, 210, 90),
           ok=(70, 200, 110), bad=(225, 80, 80), cursor=(255, 255, 255), warn=(255, 190, 60))


def fitts_id(d, w):
    return math.log2(d / w + 1.0)


def williams_rows(n):
    """Balanced Latin square (Williams design). For odd n returns 2n rows."""
    base, lo, hi = [0], 1, n - 1
    while len(base) < n:
        base.append(lo); lo += 1
        if len(base) < n:
            base.append(hi); hi -= 1
    rows = [[(b + i) % n for b in base] for i in range(n)]
    if n % 2 == 1:
        rows += [list(reversed(r)) for r in rows]
    return rows


# ---------------- input sources ----------------
class SerialInput:
    def __init__(self, port):
        from esp_console import open_port
        self._open_port = open_port
        self.port = port
        self.ser = None
        self.lock = threading.Lock()
        self.q = queue.Queue()
        self.log = []
        self.params_line = ""
        self.alive = True
        self.gray = None
        self.reopens = 0
        try:
            self.ser = open_port(port)
        except Exception as e:
            self.q.put(("#", f"serial not open yet: {e}"))
        threading.Thread(target=self._run, daemon=True).start()

    def _reopen(self):
        """The board can reset when the port first opens. Try again a few times, then stop."""
        if self.reopens >= 3:
            time.sleep(1.0)
            return
        self.reopens += 1
        with self.lock:
            old = self.ser
            self.ser = None
        try:
            if old is not None:
                old.close()
        except Exception:
            pass
        for _ in range(20):
            if not self.alive:
                return
            try:
                ser = self._open_port(self.port)
            except Exception:
                time.sleep(0.4)
                continue
            with self.lock:
                self.ser = ser
            self.q.put(("#", f"serial open {self.port}"))
            return
        self.q.put(("#", f"could not open {self.port}. Close anything else using it."))

    def _run(self):
        buf = b""; img = None
        while self.alive:
            with self.lock:
                ser = self.ser
            if ser is None:
                self._reopen()
                continue
            try:
                with self.lock:
                    if self.ser is None:
                        continue
                    waiting = self.ser.in_waiting
                    chunk = self.ser.read(waiting if waiting else 1)   # bulk read: readline() is slow on Windows
            except Exception as e:
                self.q.put(("#", f"serial error: {e}"))
                self._reopen()
                continue
            if not chunk:
                continue
            host_t = time.perf_counter()
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").rstrip("\r")
                if not line.strip():
                    continue
                if self._take_gray(line):
                    continue
                line = line.strip()
                # SETUP-ONLY camera preview ('G' dump): assembled here, never used for recognition
                if line.startswith("#IMG"):
                    img = []; continue
                if img is not None and line.startswith("#I"):
                    if len(line) == 322:
                        img.append(bytes.fromhex(line[2:]))
                    if len(img) >= 120:
                        self.q.put(("IMG", b"".join(img[:120]))); img = None
                    continue
                if img is not None:          # dump ended early (a dropped row): pad and deliver
                    if len(img) >= 110:
                        img += [img[-1]] * (120 - len(img)); self.q.put(("IMG", b"".join(img)))
                    img = None
                self._parse(line, host_t)

    def _take_gray(self, line):
        """40x30 setup preview. Cursor lines in the same burst must not throw the frame away."""
        if line.startswith("#GRAY"):
            self.gray = []
            return True
        if self.gray is None or not line.startswith("# "):
            return False
        row = line[2:]
        self.gray.append(row[:40].ljust(40))
        if len(self.gray) < 30:
            return True
        ramp = " .:-=+*#%@"
        vals = bytearray(30 * 40)
        for y, row in enumerate(self.gray[:30]):
            for x, ch in enumerate(row):
                i = ramp.find(ch)
                vals[y * 40 + x] = 0 if i < 0 else int(i * 255 / 9)
        self.q.put(("GRAY", bytes(vals)))
        self.gray = None
        return True

    def _parse(self, line, host_t):
        if True:
            p = line.split(",")
            try:
                if p[0] == "M" and len(p) >= 4:
                    self.q.put(("M", int(p[1]), int(p[2]) / 10000.0, int(p[3]) / 10000.0, host_t))
                elif p[0] == "B" and len(p) >= 4:
                    self.q.put(("B", int(p[1]), p[2] == "1", p[3], host_t))
                elif p[0] == "T" and len(p) >= 3:
                    self.q.put(("T", int(p[1]), p[2] == "1", host_t))
                elif line.startswith("#"):
                    if line.startswith("#P "):
                        self.params_line = line[3:]
                    self.q.put(("#", line))
            except ValueError:
                pass

    def send(self, cmd):
        data = (cmd + "\n").encode()
        with self.lock:
            if self.ser is None:
                return
            try:
                self.ser.write(data)
            except Exception as e:
                self.q.put(("#", f"serial write error: {e}"))

    def events(self):
        out = []
        while True:
            try:
                out.append(self.q.get_nowait())
            except queue.Empty:
                return out


class MouseInput:
    """Debug only: emulates the ESP32 stream with the system mouse."""
    def __init__(self, screen_w):
        self.sw = screen_w; self.down = False; self.params_line = "mouse-debug"

    def send(self, cmd):
        pass

    def events(self):
        t = int(time.perf_counter() * 1000); h = time.perf_counter()
        x, y = pygame.mouse.get_pos()
        ev = [("M", t, x / max(1, self.sw - 1), 0.5, h)]
        b = pygame.mouse.get_pressed()[0]
        if b != self.down:
            self.down = b; ev.append(("B", t, b, "g", h))
        return ev


# ---------------- the study ----------------
class Study:
    def __init__(self, args, screen_w, screen_h, inp):
        self.a = args; self.sw, self.sh = screen_w, screen_h; self.inp = inp
        self.cy = screen_h // 2
        self.conditions = [(d, w) for w in WIDTHS for d in DISTANCES]   # 9 (D, W)
        need = max(DISTANCES) + max(WIDTHS) + 40
        if need > screen_w:
            raise SystemExit(
                f"Screen too narrow ({screen_w}px) for D+W={need}px after fitting. "
                "Use a wider display."
            )
        digits = "".join(ch for ch in args.pid if ch.isdigit())
        pnum = int(digits) if digits else 1
        rows = williams_rows(len(self.conditions))
        self.block_order = rows[(pnum - 1) % len(rows)]
        rng = random.Random(f"practice-{args.pid}")
        # practice = one block of a single medium-difficulty condition (same structure as recorded blocks)
        self.practice_seq = [self.conditions.index(PRACTICE_CONDITION)] * PRACTICE_ATTEMPTS

        os.makedirs(DATA_DIR, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(DATA_DIR, f"{args.pid}_{stamp}.csv")
        self.meta_path = os.path.join(DATA_DIR, f"{args.pid}_{stamp}_session.json")
        self.fields = ["participant", "phase", "attempt", "block", "condition", "success_index",
                       "D_px", "W_px", "ID_bits", "direction", "start_x", "target_x", "release_x",
                       "end_error_px", "overlap", "success", "MT_ms", "MT_host_ms", "release_reason",
                       "input", "timestamp"]
        self.f = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.f, fieldnames=self.fields); self.w.writeheader(); self.f.flush()

        self.phase = "SETUP"; self.msgs = []; self.tracking = isinstance(inp, MouseInput)
        self.cursor_x = screen_w / 2; self.button = False; self.hand_y = 0.5
        self.attempt = 0; self.practice_i = 0; self.block_i = 0; self.succ_in_block = 0
        self.fail_total = 0; self.direction = 1
        self.state = "IDLE"; self.feedback_until = 0; self.last_result = None; self.hint_until = 0
        self.cal_until = 0
        self.preview = None; self.preview_req = 0.0   # setup-only camera thumbnail
        self.board_note = "Waiting for the board to boot. The picture shows up in the box on the left."
        self.esc_at = -10000                          # time of the first ESC press
        # --debug: per-frame recognition stats are printed to the console (setup/testing only)
        self.inp.send("v1" if getattr(args, "debug", False) else "v0"); self.inp.send("p")

    # ---- trial geometry ----
    def current_condition(self):
        if self.phase == "PRACTICE":
            return self.practice_seq[self.practice_i]
        return self.block_order[self.block_i]

    def setup_trial(self, keep_direction=False):
        d, w = self.conditions[self.current_condition()]
        if not keep_direction:
            self.direction = 1 if self.attempt % 2 == 0 else -1
        cx = self.sw / 2
        self.start_x = cx - self.direction * d / 2
        self.target_x = cx + self.direction * d / 2
        self.obj_x = self.start_x
        self.D, self.W = d, w
        self.state = "WAIT_GRAB"

    # ---- event handling ----
    def handle(self, ev):
        kind = ev[0]
        if kind == "GRAY":               # setup-screen picture only, not used for recognition
            if self.phase == "SETUP":
                a = np.frombuffer(ev[1], dtype=np.uint8).reshape(30, 40)
                rgb = np.repeat(a[:, :, None], 3, axis=2).transpose(1, 0, 2).copy()
                self.preview = pygame.transform.scale(pygame.surfarray.make_surface(rgb), (480, 360))
                self.board_note = "Camera is on. Ceiling should show around the hand."
            return
        if kind == "IMG":                # setup-screen picture only, not used for recognition
            if self.phase == "SETUP":
                a = np.frombuffer(ev[1], dtype=np.uint8).reshape(120, 160)
                rgb = np.repeat(a[:, :, None], 3, axis=2).transpose(1, 0, 2).copy()
                self.preview = pygame.transform.scale(pygame.surfarray.make_surface(rgb), (480, 360))
            return
        if kind == "#":
            print(ev[1], flush=True)
            if not ev[1].startswith("#V"):
                self.msgs = (self.msgs + [ev[1]])[-10:]
            text = ev[1]
            if self.preview is None:
                if text.startswith("#ready") or text.startswith("serial open"):
                    self.board_note = "Board is on. Waiting for the first picture."
                elif "serial" in text.lower() or "camera error" in text or text.startswith("could not open"):
                    self.board_note = text[:90]
            return
        if kind == "T":
            self.tracking = ev[2]; return
        if kind == "M":
            self.cursor_x = ev[2] * (self.sw - 1); self.tracking = True
            self.hand_y = ev[3]          # 0 = far from the arm-entry edge, 1 = at the edge of the camera view
            if self.state == "DRAG":
                self.obj_x = self.cursor_x + self.grab_offset
            return
        if kind == "B":
            _, t_dev, down, why, t_host = ev
            self.button = down
            if self.phase not in ("PRACTICE", "RECORDED"):
                return
            if down and self.state == "WAIT_GRAB":
                if abs(self.cursor_x - self.obj_x) <= self.W / 2:
                    self.state = "DRAG"; self.grab_offset = self.obj_x - self.cursor_x
                    self.t0_dev, self.t0_host = t_dev, t_host
                else:
                    self.hint_until = pygame.time.get_ticks() + 1200
            elif not down and self.state == "DRAG":
                self.finish_attempt(t_dev, t_host, why)

    def finish_attempt(self, t_dev, t_host, why):
        w = self.W
        ol = max(0.0, min(self.obj_x + w / 2, self.target_x + w / 2) - max(self.obj_x - w / 2, self.target_x - w / 2)) / w
        ok = ol > OVERLAP_CRITERION
        self.attempt += 1
        row = dict(participant=self.a.pid, phase=self.phase.lower(), attempt=self.attempt,
                   block=(self.block_i + 1) if self.phase == "RECORDED" else 0,
                   condition=self.current_condition(), success_index="",
                   D_px=self.D, W_px=w, ID_bits=round(fitts_id(self.D, w), 4), direction=self.direction,
                   start_x=round(self.start_x, 1), target_x=round(self.target_x, 1),
                   release_x=round(self.obj_x, 1), end_error_px=round(self.obj_x - self.target_x, 1),
                   overlap=round(ol, 4), success=int(ok), MT_ms=t_dev - self.t0_dev,
                   MT_host_ms=round((t_host - self.t0_host) * 1000, 1), release_reason=why,
                   input="mouse" if isinstance(self.inp, MouseInput) else "esp32",
                   timestamp=dt.datetime.now().isoformat(timespec="milliseconds"))
        if self.phase == "RECORDED" and ok:
            self.succ_in_block += 1; row["success_index"] = self.succ_in_block
        if not ok and self.phase == "RECORDED":
            self.fail_total += 1
        self.w.writerow(row); self.f.flush()
        self.last_result = ok; self.state = "FEEDBACK"
        self.feedback_until = pygame.time.get_ticks() + FEEDBACK_MS
        self.repeat = not ok; self.skipped = False

    def skip_attempt(self):
        """Operator key K: give up the current attempt. Logged as unsuccessful (release_reason='skip');
        in the recorded phase the same condition is repeated, so 10 successes are still required."""
        if self.phase not in ("PRACTICE", "RECORDED") or self.state not in ("WAIT_GRAB", "DRAG"):
            return
        self.attempt += 1
        row = dict(participant=self.a.pid, phase=self.phase.lower(), attempt=self.attempt,
                   block=(self.block_i + 1) if self.phase == "RECORDED" else 0,
                   condition=self.current_condition(), success_index="",
                   D_px=self.D, W_px=self.W, ID_bits=round(fitts_id(self.D, self.W), 4), direction=self.direction,
                   start_x=round(self.start_x, 1), target_x=round(self.target_x, 1),
                   release_x=round(self.obj_x, 1), end_error_px=round(self.obj_x - self.target_x, 1),
                   overlap=0.0, success=0, MT_ms="", MT_host_ms="", release_reason="skip",
                   input="mouse" if isinstance(self.inp, MouseInput) else "esp32",
                   timestamp=dt.datetime.now().isoformat(timespec="milliseconds"))
        if self.phase == "RECORDED":
            self.fail_total += 1
        self.w.writerow(row); self.f.flush()
        self.last_result = False; self.state = "FEEDBACK"; self.skipped = True
        self.feedback_until = pygame.time.get_ticks() + FEEDBACK_MS
        self.repeat = True

    def advance(self):
        """Called when feedback ends: choose the next trial / phase."""
        if self.phase == "PRACTICE":
            self.practice_i += 1
            if self.practice_i >= len(self.practice_seq):
                self.phase = "BREAK"; self.break_msg = "Practice finished. Recording starts next."
                self.next_phase = "RECORDED"; self.attempt = 0; return
            self.setup_trial(); return
        # recorded: a failed attempt repeats the same condition (same direction & positions)
        if self.repeat:
            self.setup_trial(keep_direction=True); return
        if self.succ_in_block >= SUCCESS_PER_CONDITION:
            self.block_i += 1; self.succ_in_block = 0
            if self.block_i >= len(self.block_order):
                self.phase = "DONE"; self.save_meta(); return
            self.phase = "BREAK"; self.next_phase = "RECORDED"
            self.break_msg = f"Block {self.block_i}/{len(self.block_order)} complete. Rest your arm."
            return
        self.setup_trial()

    def start_phase(self, ph):
        self.phase = ph
        if ph in ("PRACTICE", "RECORDED"):
            self.inp.send("z1")   # trials receive cursor and button events only
            self.preview = None
        if ph == "PRACTICE":
            self.practice_i = 0
        self.setup_trial()

    def save_meta(self):
        meta = dict(participant=self.a.pid, csv=os.path.basename(self.csv_path),
                    screen_px=[self.sw, self.sh], widths_px=WIDTHS, distances_px=DISTANCES,
                    bar_height_px=BAR_HEIGHT, overlap_criterion=OVERLAP_CRITERION,
                    success_per_condition=SUCCESS_PER_CONDITION, practice_attempts=PRACTICE_ATTEMPTS,
                    block_order=[dict(condition=c, D=self.conditions[c][0], W=self.conditions[c][1])
                                 for c in self.block_order],
                    esp32_params=self.inp.params_line,
                    input="mouse" if isinstance(self.inp, MouseInput) else "esp32",
                    recorded_failures=self.fail_total,
                    saved=dt.datetime.now().isoformat(timespec="seconds"))
        with open(self.meta_path, "w", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)

    # ---- drawing ----
    def draw(self, scr, fonts):
        big, mid, small = fonts
        scr.fill(COL["bg"])
        now = pygame.time.get_ticks()
        if self.phase == "SETUP":
            self._draw_setup(scr, big, mid, small, now)
        elif self.phase in ("BREAK", "DONE"):
            msg = self.break_msg if self.phase == "BREAK" else "All blocks complete - thank you!"
            sub = "Press SPACE to continue." if self.phase == "BREAK" else f"Data saved to {os.path.basename(self.csv_path)}. Press ESC."
            self._centered(scr, big, msg, self.cy - 50, COL["text"])
            self._centered(scr, mid, sub, self.cy + 20, COL["text"])
        else:
            self._draw_trial(scr, big, mid, small, now)
        if now - self.esc_at < 2000:
            self._centered(scr, mid, "Press ESC again to quit. Data so far is saved.", self.sh - mid.get_height() - 8, COL["bad"])
        quit_label = "K = skip attempt (counts as a miss)    ESC = quit" if self.phase in ("PRACTICE", "RECORDED") else "ESC = quit"
        img = small.render(quit_label, True, COL["dim"])
        scr.blit(img, (self.sw - img.get_width() - 16, 10))

    def _draw_setup(self, scr, big, mid, small, now):
        lay = self._setup_layout(big)
        self._blit_wrapped(scr, big, "Fitts' Law study", 16, 8, self.sw - 220, COL["text"], big.get_linesize())
        lines = ["Elbow on the desk. Move only the forearm.",
                 "1. OPEN hand left/right. Cursor follows. Press M if it is backwards.",
                 "2. Press C, sweep for 6 s, until the cursor reaches both edges.",
                 "3. Hold still. The cursor should stay still.",
                 "4. FIST fills the dot. OPEN hand makes it a ring.",
                 "Task: cursor on the ORANGE bar, FIST to grab, drag onto the BLUE bar, OPEN to drop.",
                 "Work quickly and accurately. ENTER starts practice and turns the camera off.",
                 "Keys: B background (hand out)  C sweep  M mirror  0-3 arm edge  S save"]
        line_h = small.get_linesize() + 2
        y = lay["text"][1]
        for s in lines:
            if y + line_h > lay["lane_y"] - 8:
                break
            y = self._blit_wrapped(scr, small, s, lay["text"][0], y, lay["text"][2], COL["text"], line_h)
        if now < self.cal_until and y + line_h < lay["lane_y"] - 8:
            left = (self.cal_until - now) / 1000
            y = self._blit_wrapped(scr, small, f"Calibrating {left:.0f} s. Sweep the OPEN hand to both sides.",
                                   lay["text"][0], y + 6, lay["text"][2], COL["warn"], line_h)
        elif self.cal_until and now < self.cal_until + 4000 and y + line_h < lay["lane_y"] - 8:
            y = self._blit_wrapped(scr, small, "Sweep saved in memory. Press S to keep it, then check both edges.",
                                   lay["text"][0], y + 6, lay["text"][2], COL["ok"], line_h)
        if self.tracking and y + line_h < lay["lane_y"] - 8:
            st = "Hand: FIST (grab)" if self.button else "Hand: OPEN"
            self._blit_wrapped(scr, mid, st, lay["text"][0], y + 8, lay["text"][2],
                               COL["obj"] if self.button else COL["ok"], mid.get_linesize())
        pygame.draw.rect(scr, COL["target"], (16, lay["lane_y"], self.sw - 32, 8), 1)
        x = int(self.cursor_x)
        x = min(self.sw - 20, max(20, x))
        pygame.draw.circle(scr, COL["cursor"], (x, lay["lane_y"] + 4), 10 if self.button else 7, 0 if self.button else 2)
        msg_y = lay["lane_y"] + 16
        for s in self.msgs[-3:]:
            if msg_y + small.get_linesize() > self.sh - 4:
                break
            scr.blit(small.render(s[:110], True, COL["dim"]), (16, msg_y))
            msg_y += small.get_linesize()
        self._draw_camera(scr, small, lay["cam"])

    def _draw_trial(self, scr, big, mid, small, now):
        w, top = self.W, self.cy - BAR_HEIGHT / 2
        pygame.draw.rect(scr, COL["target"], (self.target_x - w / 2, top, w, BAR_HEIGHT))
        pygame.draw.rect(scr, COL["target_edge"], (self.target_x - w / 2, top, w, BAR_HEIGHT), 3)
        oc = COL["obj_drag"] if self.state == "DRAG" else COL["obj"]
        if self.state == "FEEDBACK":
            oc = COL["ok"] if self.last_result else COL["bad"]
        pygame.draw.rect(scr, oc, (self.obj_x - w / 2, top, w, BAR_HEIGHT))
        if self.state == "FEEDBACK":
            s = "HIT" if self.last_result else ("SKIPPED - counted as a miss" if getattr(self, "skipped", False) else "MISS - try again")
            self._centered(scr, big, s, self._above_bars(top, big.get_height()), COL["ok"] if self.last_result else COL["bad"])
        if self.button and self.state == "WAIT_GRAB":
            self._centered(scr, small, "Fist closed off the orange bar. Open your hand, aim, then make a fist.",
                           self._above_bars(top, small.get_height()), COL["warn"])
        elif now < self.hint_until:
            self._centered(scr, small, "Grab while the cursor is on the orange bar.",
                           min(self.sh - 36, top + BAR_HEIGHT + 12), COL["warn"])
        if self.phase == "PRACTICE":
            st = f"PRACTICE {self.practice_i + 1}/{len(self.practice_seq)}"
        else:
            st = f"Block {self.block_i + 1}/{len(self.block_order)}   hits {self.succ_in_block}/{SUCCESS_PER_CONDITION}"
        scr.blit(small.render(st, True, COL["dim"]), (16, 8))
        if self.phase == "PRACTICE":
            self._centered(scr, small, "ORANGE bar, FIST to grab, drag onto BLUE, OPEN to drop.",
                           self.sh - small.get_height() - 8, COL["dim"])
        x = int(self.cursor_x)
        pygame.draw.line(scr, COL["cursor"], (x, self.cy - 24), (x, self.cy + 24), 3)
        pygame.draw.circle(scr, COL["cursor"], (x, self.cy), 11 if self.button else 7, 0 if self.button else 2)
        if not self.tracking:
            self._centered(scr, small, "Hand not detected. Move your hand over the camera.",
                           self._above_bars(top, small.get_height()), COL["warn"])
        elif self.hand_y > 0.66 and not self.button:
            self._centered(scr, small, "Reach further forward. Your hand is at the edge of the camera.",
                           self._above_bars(top, small.get_height()), COL["warn"])
        gh = min(220, max(80, int(BAR_HEIGHT * 0.7)))
        gx, gy0 = 8, int(self.cy - gh / 2)
        pygame.draw.rect(scr, COL["dim"], (gx, gy0, 8, gh), 1)
        pygame.draw.rect(scr, COL["ok"], (gx, gy0, 8, int(gh * 0.66)), 0)
        if self.tracking:
            pygame.draw.circle(scr, COL["text"], (gx + 4, gy0 + int(gh * min(1.0, self.hand_y))), 6)

    def _above_bars(self, top, text_h):
        y = top - text_h - 10
        if y < 28:
            y = 28
        return y

    def _setup_layout(self, title_font):
        margin = 16
        top = 12 + title_font.get_linesize() + 8
        footer = 72 if self.sh < 820 else 100
        lane_y = self.sh - footer
        room_h = lane_y - top - 8
        side = self.sw >= 1100 and room_h >= 280
        if side:
            pw = min(400, int(self.sw * 0.30))
            ph = min(int(pw * 0.75), room_h - 24)
            return dict(cam=(margin, top, pw, max(140, ph)),
                        text=(margin + pw + 20, top, self.sw - (margin + pw + 20) - margin),
                        lane_y=lane_y)
        pw = min(460, self.sw - 2 * margin)
        ph = min(200, max(120, int(room_h * 0.36)))
        return dict(cam=((self.sw - pw) // 2, top, pw, ph),
                    text=(margin, top + ph + 28, self.sw - 2 * margin),
                    lane_y=lane_y)

    def _centered(self, scr, font, text, y, color):
        max_w = self.sw - 48
        words, lines, line = text.split(" "), [], ""
        for word in words:
            trial = word if not line else line + " " + word
            if font.size(trial)[0] <= max_w or not line:
                line = trial
            else:
                lines.append(line)
                line = word
        if line:
            lines.append(line)
        for i, row in enumerate(lines[:3]):
            img = font.render(row, True, color)
            scr.blit(img, (self.sw / 2 - img.get_width() / 2, y + i * (font.get_linesize() + 1)))

    def _blit_wrapped(self, scr, font, text, x, y, max_w, color, line_h):
        if y + line_h > self.sh - 4:
            return y
        if not text:
            return y + line_h
        line = ""
        for word in text.split(" "):
            trial = word if not line else line + " " + word
            if font.size(trial)[0] <= max(40, max_w) or not line:
                line = trial
            else:
                scr.blit(font.render(line, True, color), (x, y))
                y += line_h
                line = word
        if line and y + line_h <= self.sh - 2:
            scr.blit(font.render(line, True, color), (x, y))
            y += line_h
        return y

    def _draw_camera(self, scr, small, box):
        px, py, pw, ph = box
        pygame.draw.rect(scr, (12, 12, 14), (px, py, pw, ph))
        if self.preview is not None:
            scr.blit(pygame.transform.scale(self.preview, (pw, ph)), (px, py))
            cap = "Ceiling around the hand. Fist fills the dot."
            if self.tracking and self.hand_y > 0.66 and not self.button:
                cap = "Hand is too low in the picture. Raise it."
        else:
            self._blit_wrapped(scr, small, self.board_note, px + 12, py + 16, pw - 24, COL["text"], small.get_linesize() + 2)
            cap = "Camera. It can take a few seconds to appear."
        pygame.draw.rect(scr, COL["obj"], (px, py, pw, ph), 3)
        if py + ph + small.get_linesize() < self.sh - 4:
            scr.blit(small.render(cap, True, COL["dim"]), (px, py + ph + 4))

    def close(self):
        self.save_meta(); self.f.close()


# USB-HID scancodes (layout- and IME-independent) for the operator keys
SCAN_KEYS = {14: pygame.K_k, 5: pygame.K_b, 6: pygame.K_c, 16: pygame.K_m, 22: pygame.K_s, 19: pygame.K_p,
             39: pygame.K_0, 30: pygame.K_1, 31: pygame.K_2, 32: pygame.K_3,
             40: pygame.K_RETURN, 88: pygame.K_RETURN, 41: pygame.K_ESCAPE, 44: pygame.K_SPACE}


def ui_fonts(screen_h):
    """Sizes follow the screen. Font names cover Windows and macOS."""
    scale = min(1.0, max(0.78, screen_h / 1000.0))
    sans = "segoeui,helveticaneue,helvetica,arial,freesans,dejavusans"
    mono = "consolas,menlo,monaco,dejavusansmono,couriernew,freemono"
    return (pygame.font.SysFont(sans, max(22, int(34 * scale)), bold=True),
            pygame.font.SysFont(sans, max(16, int(22 * scale))),
            pygame.font.SysFont(mono, max(13, int(16 * scale))))


def widest_monitor():
    """Prefer the largest attached screen so the fullscreen task is not stuck on the laptop panel."""
    if sys.platform != "win32":
        return None
    import ctypes
    from ctypes import wintypes
    found = []

    class RECT(ctypes.Structure):
        _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long),
                    ("right", ctypes.c_long), ("bottom", ctypes.c_long)]

    proc = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(RECT), ctypes.c_double)

    def _cb(hmon, hdc, rect, data):
        r = rect.contents
        found.append((r.right - r.left, r.bottom - r.top, r.left, r.top))
        return 1

    ctypes.windll.user32.EnumDisplayMonitors(None, None, proc(_cb), 0)
    if not found:
        return None
    external = [m for m in found if m[2] or m[3]]
    width, height, x, y = max(external or found)
    os.environ["SDL_VIDEO_WINDOW_POS"] = f"{x},{y}"
    return width, height


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pid", required=True, help="anonymous participant id, e.g. P01")
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--mouse", action="store_true", help="debug with the system mouse (not for study data)")
    ap.add_argument("--windowed", action="store_true")
    ap.add_argument("--debug", action="store_true", help="print per-frame ESP recognition stats (testing only)")
    a = ap.parse_args()
    big = widest_monitor()

    pygame.init()
    if a.windowed:
        scr = pygame.display.set_mode((1800, 1000))
    elif big:
        scr = pygame.display.set_mode((big[0], big[1]), pygame.NOFRAME)
    else:
        scr = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    sw, sh = scr.get_size()
    pygame.key.stop_text_input()          # no IME composition: keys arrive as plain KEYDOWN events
    pygame.display.set_caption("Fitts ESP32 study")
    pygame.mouse.set_visible(a.mouse)
    fonts = ui_fonts(sh)
    scaled = fit_task_to_screen(sw, sh)
    inp = MouseInput(sw) if a.mouse else SerialInput(a.port)
    st = Study(a, sw, sh, inp)
    print(f"screen {sw}x{sh}  widths {WIDTHS}  distances {DISTANCES}  block order {st.block_order}  log -> {st.csv_path}")
    if scaled:
        print("Display is under 2440 px wide, so the distances were reduced to fit. "
              "Use this same screen, fullscreen, for every participant.")
    clock = pygame.time.Clock(); running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                # use the physical key (scancode) so a Chinese/Japanese IME cannot swallow letter keys
                k = SCAN_KEYS.get(e.scancode, e.key)
                if k == pygame.K_ESCAPE:     # double-press to quit (prevents accidental exits mid-study)
                    if st.phase == "DONE" or pygame.time.get_ticks() - st.esc_at < 2000:
                        running = False
                    else:
                        st.esc_at = pygame.time.get_ticks()
                elif st.phase == "SETUP":
                    if k == pygame.K_RETURN:
                        st.start_phase("PRACTICE")
                    elif k == pygame.K_b: inp.send("b")
                    elif k == pygame.K_c:
                        inp.send("c"); st.cal_until = pygame.time.get_ticks() + 6000
                    elif k == pygame.K_m: inp.send("m")
                    elif k == pygame.K_s: inp.send("s")
                    elif k == pygame.K_p: inp.send("p")
                    elif pygame.K_0 <= k <= pygame.K_3: inp.send(f"o{k - pygame.K_0}")
                elif st.phase in ("PRACTICE", "RECORDED") and k == pygame.K_k:
                    st.skip_attempt()
                elif st.phase == "BREAK" and k == pygame.K_SPACE:
                    st.start_phase(st.next_phase)
        # Small picture on the setup screen only. Practice and recorded trials do not request it.
        if st.phase == "SETUP" and isinstance(inp, SerialInput) and pygame.time.get_ticks() > st.cal_until:
            if time.perf_counter() - st.preview_req > 0.35:
                inp.send("g"); st.preview_req = time.perf_counter()
        for ev in inp.events():
            st.handle(ev)
        if st.state == "FEEDBACK" and pygame.time.get_ticks() >= st.feedback_until and st.phase in ("PRACTICE", "RECORDED"):
            st.advance()
        st.draw(scr, fonts)
        pygame.display.flip()
        clock.tick(120)
    st.close(); pygame.quit()
    print(f"saved {st.csv_path}")


if __name__ == "__main__":
    main()
