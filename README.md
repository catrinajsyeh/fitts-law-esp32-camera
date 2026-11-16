# Fitts' Law Drag-and-Drop Study with an ESP32-S3 Camera Hand Interface

```
firmware/FittsHandTracker/FittsHandTracker.ino   ESP32 firmware (all vision + gesture recognition)
laptop_app/fitts_app.py                          study application (pygame): rendering, timing, logging
laptop_app/esp_console.py                        serial console for setup / calibration
analysis/analyze.py                              plots + regression + error counts -> analysis/out/
data/                                            CSV trial logs (anonymous IDs) + session JSON
```

## 1. Hardware and wiring
* **Board:** ESP32-S3 on COM3 (USB-Enhanced-SERIAL CH343), 8 MB flash, no PSRAM, OV3660. Camera pins are the ESP32-S3-EYE set (XCLK 15, SDA 4, SCL 5, D0–D7 11/9/8/10/12/18/17/16, VSYNC 6, HREF 7, PCLK 13). This is not the XIAO board.
* **Link:** one USB cable. Serial is UART0 on the CH343 adapter at 115200. Do not enable USB CDC.
* **Mounting:** the camera lies flat and looks **straight up** at a plain white ceiling. No lamp in the picture. The bare hand is above the lens, palm toward the lens, elbow on the desk. The forearm enters from one edge. Keep the hand high enough that ceiling shows around it.

## 2. Dependencies
* PlatformIO with the Espressif 32 platform and the Arduino framework. Board target: `esp32-s3-devkitc-1`, 8 MB flash, DIO mode. This board has no PSRAM. Do not enable USB CDC; serial is the CH343 adapter on COM3.
  The camera driver (`esp_camera.h`) and `Preferences` ship with the core. `pio.exe` is blocked on this machine, so use `python -m platformio`.
* Python 3.10+ with `pygame-ce`, `pyserial`, `numpy`, `matplotlib`, and `fpdf2`:
  ```
  python -m pip install -r requirements.txt
  ```

## 3. Build and flash
From the `firmware` folder, with the study window closed so COM3 is free:
```
python -m platformio run -e esp32s3cam -t upload
```
On boot the firmware spends 2 s with auto-exposure on, then **locks exposure and gain**. Next it averages 16 frames
into a background model, so **keep your hand out of view for the first ~4 s after reset**.

## 4. How recognition works (on the ESP32)
| Stage | Detail |
|---|---|
| Capture | 160×120 grayscale (QQVGA). This board has no PSRAM, so the frame buffer is in DRAM |
| Segmentation | per-pixel `|frame − background| > diffT` (default 22/255; polarity configurable). The background adapts slowly (1/32 every 4th frame) on non-foreground pixels only |
| Clean-up | 3×3 majority filter: removes speckle and fills 1-px holes |
| Hand blob | largest 4-connected component; rejected if < `minArea` (150 px) or > 70 % of frame |
| Canonical frame | image rotated so the arm enters from the bottom (`orient`); row 0 is farthest from the arm |
| Cursor feature | x-centroid of the first `bandRows`=30 rows of the distal hand band |
| Gesture | **fist solidity**, not a finger count. Solidity is filled pixels divided by the span of the distal hand. A grab starts at solidity ≥ 76 and is held while solidity stays ≥ 70. An open hand (solidity ≤ 60) releases. Values in between are ambiguous and need 3 confirming frames. A blob under 100 px cannot start a grab |

**Cursor mapping: absolute.** `u = (x_centroid − xmin)/(xmax − xmin)`, clamped to [0, 1] and mirrored if needed,
then scaled to 0–10000. The laptop maps it to `u·(screen_width−1)`, so a hand sweep across the calibrated range
covers the full screen width (about 100 camera px → 2560 screen px, i.e. ~25 screen px per camera pixel; the
sub-pixel centroid plus filtering gives finer resolution than that).
**Smoothing:** One-Euro filter (min cutoff 1.0 Hz, β = 0.0007 per unit/s). The cursor is steady when the hand is
still and has little lag during fast moves.

**Gesture-induced cursor shift.** Opening the fist moves the tracked point. While a block is held, the cursor follows the hand only while the fist stays solid. As soon as solidity drops by 10 points or falls below 70, the block stays where the fist was until the release is confirmed or the fist tightens again (solidity ≥ 76). The button event is sent at that held position.

**Tracking loss.** Frames with no valid hand do not move the cursor. Single-frame centroid jumps > 25 % of the range
are rejected as outliers. After 150 ms without a hand the device sends `T,0`. If the hand was grabbing for
500 ms without being seen, a forced release `B,…,0,l` is sent so an object is never left engaged. Re-acquisition
needs 3 stable frames. The cursor then **continues from where it was** (offset re-anchored, no jump).

## 5. Communication (ESP32 → laptop)
Only recognised input events are sent during practice and recorded trials. A small camera picture is shown on the setup screen only, so the hand can be placed before the first trial. It is not used for recognition, and it turns off when practice starts.
```
M,<t_ms>,<x>,<y>        cursor position, 0..10000   (every processed frame while tracking)
B,<t_ms>,<1|0>,<g|l>    button press / release (g = gesture, l = forced by tracking loss)
T,<t_ms>,<1|0>          tracking acquired / lost
#...                    human-readable status (ignored by the app)
```
`t_ms` is the ESP32 `millis()` timestamp of the frame. Laptop → ESP32 commands (one per line):

| cmd | action | cmd | action |
|---|---|---|---|
| `b` | re-capture background (hand out of view) | `c` | 6-s range calibration |
| `o0..o3` | arm enters from bottom/top/left/right | `m` | toggle mirror |
| `t<n>` | difference threshold | `q<n>` | polarity 0 abs, 1 hand brighter, 2 darker |
| `a<n>` | min hand area | `v0/v1` | per-frame debug stats |
| `d` | print one ASCII mask (setup only) | `p` | print parameters |
| `s` | save parameters to flash | `r` | restore defaults |
| `e` | re-run exposure lock + background | | |

`d` and `v` are setup/diagnostic aids typed by the experimenter. They are never used for recognition and are off
during the study (the app sends `v0` at start).

## 6. Calibration procedure (once, before the first participant; then keep fixed)
1. Point the camera straight up at a plain white ceiling. Keep lights fixed and keep lamps out of the picture.
2. Reset the board with your hand out of view so it can lock exposure and capture the empty ceiling.
3. Hold the bare hand above the lens, palm toward the camera, high enough that ceiling shows around the hand.
4. A fist (solidity at least 76) grabs. Opening the hand (solidity at most 60) releases. A small blob cannot start a grab.
5. Press C and sweep the open hand slowly fully left and right for 6 s. The cursor should match that sweep (press M if reversed).
6. Press S to save. Leave these thresholds fixed for every participant.

## 7. Running the study

The person collecting the trials should follow [PARTNER.md](PARTNER.md).

From this project folder:

```
python -u laptop_app/fitts_app.py --pid P01 --port COM3
```
* **Setup screen:** live cursor, ESP messages, and operator keys (B background, C calibrate, M mirror, 0–3 orientation,
  S save). Read the instructions on screen to the participant, then press **ENTER**.
* **Practice:** 5 attempts of the middle condition (D = 1080 px, W = 240 px). Logged with `phase=practice`, not plotted. The camera picture is off.
* **Recorded:** 9 blocks, one per (D, W) condition, ordered by a balanced Latin square (Williams design, row
  chosen from the participant number). Each block continues until **10 successful trials**. A failed attempt repeats
  the same condition immediately, and is logged but not counted. SPACE continues after each block break.
* **Task:** an orange object bar and a blue target bar (same width W, height 420 px, centres on the same horizontal
  line, centre distance D) sit symmetric about the screen centre. Direction alternates left/right. The participant
  moves the cursor onto the object, makes a fist (grab), drags, and opens the hand (release).
* **Timing:** MT = ESP32 timestamp of release − ESP32 timestamp of grab (so USB jitter is excluded). Laptop
  `perf_counter` MT is logged too.
* **Success:** horizontal overlap / W > 0.80 (equal heights and aligned centres, so this equals the area overlap).
  Green HIT or red MISS feedback is shown for 0.7 s.
* Conditions: W ∈ {120, 240, 480} px and D ∈ {480, 1080, 1920} px on a display at least 2440 px wide. A narrower display shortens the distances so the bars still fit. Use that same screen for every participant.

`--mouse` runs the same app with the system mouse for debugging only. Those logs have `input=mouse` and
are ignored by the analysis.

## 8. Data format

Starting the app always creates a **new** file. It never opens or fills in an older one.

```
python -u laptop_app/fitts_app.py --pid P01 --port COM3
```

That command writes `data/P01_<year><month><day>_<hour><minute><second>.csv`, using the time the window opened. `P02` and `P03` each get their own file the same way. Every practice drop and every recorded drop is added to that file as it happens, including misses. Pressing Esc twice quits; the file is already saved.

An old CSV is left untouched. `data/P01_20261116_013942.csv` only has the column names from a window that was closed before anyone dropped a bar. Delete it. Do not pack it in the zip as if it were a study.

Each row is one attempt:

`participant, phase, attempt, block, condition, success_index, D_px, W_px, ID_bits, direction, start_x, target_x, release_x, end_error_px, overlap, success, MT_ms, MT_host_ms, release_reason, input, timestamp`

`participant` is the anonymous id (`P01`). `attempt` is the attempt number. `D_px`, `W_px`, and `ID_bits` are the distance, the width, and the index of difficulty. `MT_ms` is the movement time in milliseconds. `success` is 1 for a hit and 0 for a miss.

`data/<PID>_<timestamp>_session.json` holds the screen size, conditions, block order, and ESP32 parameters.
