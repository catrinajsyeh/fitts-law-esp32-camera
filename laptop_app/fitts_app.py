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
        self.ser = open_port(port)
        self.q = queue.Queue()
        self.log = []
        self.params_line = ""
        self.alive = True
        self.gray = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        buf = b""; img = None
        while self.alive:
            try:
                chunk = self.ser.read(max(1, self.ser.in_waiting))   # bulk read: readline() is slow on Windows
            except Exception as e:
                self.q.put(("#", f"serial error: {e}")); time.sleep(0.5); continue
            if not chunk:
                continue
            host_t = time.perf_counter()
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").rstrip("\r")
                if not line.strip():
                    continue
                if line.startswith("#GRAY") or self.gray is not None:
                    self._take_gray(line)
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
        """40x30 setup preview. About 1.3 KB, so the cursor lines are not stuck behind it."""
        if line.startswith("#GRAY"):
            self.gray = []
            return
        if not line.startswith("#"):
            self.gray = None
            self._parse(line.strip(), time.perf_counter())
            return
        row = line[2:] if line.startswith("# ") else line[1:]
        self.gray.append(row[:40].ljust(40))
        if len(self.gray) < 30:
            return
        ramp = " .:-=+*#%@"
        vals = bytearray(30 * 40)
        for y, row in enumerate(self.gray[:30]):
            for x, ch in enumerate(row):
                i = ramp.find(ch)
                vals[y * 40 + x] = 0 if i < 0 else int(i * 255 / 9)
        self.q.put(("GRAY", bytes(vals)))
        self.gray = None

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
        self.ser.write((cmd + "\n").encode())

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
            lines = ["Fitts' Law study - ESP32 camera hand interface", "",
                     "SETUP CHECK (rest your elbow on the desk, move only the forearm):",
                     "  1. Move the OPEN hand left/right: the cursor should follow in the same direction (press M if reversed).",
                     "  2. A comfortable sweep should reach both screen edges (press C, then sweep the open hand for 6 s).",
                     "  3. Hold the hand still: the cursor should stay still.",
                     "  4. Make a FIST: the cursor dot fills in (GRAB).  OPEN the hand: it becomes a ring (RELEASE).",
                     "",
                     "TASK: move the cursor onto the ORANGE bar, make a FIST to grab it, drag it onto the BLUE bar,",
                     "      then OPEN the hand to drop it.  Work as quickly and accurately as you can.",
                     "",
                     "ENTER = start practice. The camera picture turns off, and only the cursor remains.",
                     "Operator keys:  B background (hand out of view)   C calibrate   M mirror   0-3 arm edge   S save"]
            for i, s in enumerate(lines):
                scr.blit((big if i == 0 else mid).render(s, True, COL["text"]), (60, 60 + i * 46))
            if self.preview is not None:
                px = self.sw - 480 - 40
                scr.blit(self.preview, (px, 60))
                pygame.draw.rect(scr, COL["dim"], (px, 60, 480, 360), 1)
                scr.blit(small.render("Raise the hand until ceiling shows around it. Fist fills the dot.", True, COL["dim"]), (px, 428))
            if now < self.cal_until:
                left = (self.cal_until - now) / 1000
                img = big.render(f"CALIBRATING {left:.0f} s - sweep the OPEN hand slowly to your far LEFT and far RIGHT",
                                 True, COL["warn"])
                scr.blit(img, (self.sw / 2 - img.get_width() / 2, self.cy - 110))
            elif self.cal_until and now < self.cal_until + 4000:
                img = mid.render("Calibration done - press S to save, then check the cursor reaches both edges",
                                 True, COL["ok"])
                scr.blit(img, (self.sw / 2 - img.get_width() / 2, self.cy - 110))
            if self.tracking:
                st = "GRAB (fist)" if self.button else "OPEN hand (hover)"
                img = big.render(f"Hand: {st}", True, COL["ok"] if not self.button else COL["obj"])
                scr.blit(img, (self.sw / 2 - img.get_width() / 2, self.cy + 70))
            for i, s in enumerate(self.msgs):
                scr.blit(small.render(s, True, COL["dim"]), (60, self.sh - 320 + i * 28))
            pygame.draw.rect(scr, COL["target"], (0, self.cy - 6, self.sw, 12), 1)
        elif self.phase in ("BREAK", "DONE"):
            msg = self.break_msg if self.phase == "BREAK" else "All blocks complete - thank you!"
            sub = "Press SPACE to continue." if self.phase == "BREAK" else f"Data saved to {os.path.basename(self.csv_path)}. Press ESC."
            for i, (s, f) in enumerate([(msg, big), (sub, mid)]):
                img = f.render(s, True, COL["text"]); scr.blit(img, (self.sw / 2 - img.get_width() / 2, self.cy - 80 + i * 70))
        else:
            w, top = self.W, self.cy - BAR_HEIGHT / 2
            pygame.draw.rect(scr, COL["target"], (self.target_x - w / 2, top, w, BAR_HEIGHT))
            pygame.draw.rect(scr, COL["target_edge"], (self.target_x - w / 2, top, w, BAR_HEIGHT), 3)
            oc = COL["obj_drag"] if self.state == "DRAG" else COL["obj"]
            if self.state == "FEEDBACK":
                oc = COL["ok"] if self.last_result else COL["bad"]
            pygame.draw.rect(scr, oc, (self.obj_x - w / 2, top, w, BAR_HEIGHT))
            if self.state == "FEEDBACK":
                s = "HIT" if self.last_result else ("SKIPPED - counted as a miss" if getattr(self, "skipped", False) else "MISS - try again")
                img = big.render(s, True, COL["ok"] if self.last_result else COL["bad"])
                scr.blit(img, (self.sw / 2 - img.get_width() / 2, top - 110))
            if self.button and self.state == "WAIT_GRAB":
                img = mid.render("Fist closed OFF the orange bar - OPEN your hand, aim at the orange bar, then make a fist",
                                 True, COL["warn"])
                scr.blit(img, (self.sw / 2 - img.get_width() / 2, top - 70))
            elif now < self.hint_until:
                img = mid.render("Grab while the cursor is on the orange bar", True, COL["warn"])
                scr.blit(img, (self.sw / 2 - img.get_width() / 2, top + BAR_HEIGHT + 40))
            if self.phase == "PRACTICE":
                st = f"PRACTICE {self.practice_i + 1}/{len(self.practice_seq)}"
            else:
                st = f"Block {self.block_i + 1}/{len(self.block_order)}   hits {self.succ_in_block}/{SUCCESS_PER_CONDITION}"
            scr.blit(small.render(st, True, COL["dim"]), (30, 20))
            if self.phase == "PRACTICE":
                img = mid.render("Cursor onto ORANGE bar -> FIST (grab) -> drag onto BLUE bar -> OPEN hand (drop)",
                                 True, COL["dim"])
                scr.blit(img, (self.sw / 2 - img.get_width() / 2, self.sh - 120))
        if now - self.esc_at < 2000:
            q = big.render("Press ESC again to QUIT (data so far is saved)", True, COL["bad"])
            scr.blit(q, (self.sw / 2 - q.get_width() / 2, self.sh - 220))
        img = small.render("K = skip attempt (counts as a miss)   ESC = quit"
                           if self.phase in ("PRACTICE", "RECORDED") else "ESC = quit", True, COL["dim"])
        scr.blit(img, (self.sw - img.get_width() - 30, 20))
        # cursor (1-D: drawn on the common centre line of object and target)
        x = int(self.cursor_x)
        pygame.draw.line(scr, COL["cursor"], (x, self.cy - 30), (x, self.cy + 30), 3)
        pygame.draw.circle(scr, COL["cursor"], (x, self.cy), 12 if self.button else 8, 0 if self.button else 2)
        if not self.tracking:
            img = mid.render("HAND NOT DETECTED - move your hand over the camera", True, COL["warn"])
            scr.blit(img, (self.sw / 2 - img.get_width() / 2, 70))
        elif self.hand_y > 0.66 and not self.button:     # only judged while the hand is open

            img = big.render("Reach FURTHER forward - your hand is at the edge of the camera view", True, COL["warn"])
            scr.blit(img, (self.sw / 2 - img.get_width() / 2, 70))
        # hand-position gauge (left edge): green zone = hand well inside the camera view
        gx, gy0, gh = 14, self.cy - 150, 300
        pygame.draw.rect(scr, COL["dim"], (gx, gy0, 10, gh), 1)
        pygame.draw.rect(scr, COL["ok"], (gx, gy0, 10, int(gh * 0.66)), 0)
        if self.tracking:
            pygame.draw.circle(scr, COL["text"], (gx + 5, gy0 + int(gh * min(1.0, self.hand_y))), 8)

    def close(self):
        self.save_meta(); self.f.close()


# USB-HID scancodes (layout- and IME-independent) for the operator keys
SCAN_KEYS = {14: pygame.K_k, 5: pygame.K_b, 6: pygame.K_c, 16: pygame.K_m, 22: pygame.K_s, 19: pygame.K_p,
             39: pygame.K_0, 30: pygame.K_1, 31: pygame.K_2, 32: pygame.K_3,
             40: pygame.K_RETURN, 88: pygame.K_RETURN, 41: pygame.K_ESCAPE, 44: pygame.K_SPACE}


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
    fonts = (pygame.font.SysFont("segoeui", 44, bold=True), pygame.font.SysFont("segoeui", 30),
             pygame.font.SysFont("consolas", 22))
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
