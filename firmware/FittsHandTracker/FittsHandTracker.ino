/*
 * FittsHandTracker - camera-based hand input device for a 1-D Fitts' Law study
 * Board : ESP32-S3 camera on the CH343 USB-UART (COM3). Pins are the ESP32-S3-EYE set.
 * Core  : Arduino-ESP32 3.x. UART0 serial, DIO flash. Frame buffers use PSRAM when it exists.
 *
 * ALL image processing runs on the ESP32:
 *   grayscale QQVGA frame -> background subtraction -> 3x3 median -> largest
 *   connected component (the hand) -> per-row run analysis (finger count) +
 *   centroid of the distal "hand band" -> calibrated absolute mapping ->
 *   One-Euro filter -> grab/release state machine with roll-back.
 *
 * Only recognised input events leave the board (115200 text lines on UART0):
 *   M,<t_ms>,<x>,<y>          cursor position, x,y in 0..10000 (normalised)
 *   B,<t_ms>,<1|0>,<g|l>      button press(1)/release(0); g=gesture, l=tracking loss
 *   T,<t_ms>,<1|0>            hand tracking acquired(1) / lost(0)
 *   #...                      human-readable status / debug (ignored by the app)
 *
 * Gesture: OPEN spread hand (>=3 separated fingers) = button up (hover)
 *          FIST (single blob, <=1 run)              = button down (grab)
 *
 * Serial commands (single characters, optional number), see README:
 *   b  re-capture background (keep hand out of view)
 *   c  calibrate horizontal range (sweep hand left<->right for 6 s)
 *   o<n> arm-entry edge 0=bottom 1=top 2=left 3=right    m  toggle mirror
 *   t<n> diff threshold    q<n> polarity 0=abs 1=brighter 2=darker
 *   v  verbose per-frame stats on/off     d  dump ASCII mask (setup only)
 *   p  print parameters   s  save to flash   r  reset defaults   e  re-lock exposure
 */
#include "esp_camera.h"
#include <Preferences.h>

// ---------------- ESP32-S3-EYE camera pins (this CH343 board) ----------------
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13
#define LED_PIN        -1

// ---------------- image buffers ----------------
static const int IW = 160, IH = 120, NPIX = IW * IH;
static uint16_t bg16[NPIX];         // background, x16 fixed point
static uint8_t  m1[NPIX], m2[NPIX]; // canonical foreground masks
static int16_t  lab[NPIX];          // component labels
static uint16_t bfsq[NPIX];         // BFS queue
static int CW = IW, CH = IH;        // canonical width/height (depends on orientation)
static int rowCnt[160], rowRuns[160], rowSpan[160];
static int SOL_FIST = 76, SOL_OPEN = 60, SOL_HOLD = 70;   // grab >= 76, keep holding >= 70, release <= 60
static int MIN_GRAB_AREA = 100;           // px of hand area needed before a fist can grab
static int MIN_RUN = 2;   // a "finger" run must be >= this many px wide
static float rowSum[160];
static int16_t gBestLab = 0;       // label of the hand component in the last frame

// ---------------- parameters (persisted in NVS) ----------------
struct Params {
  uint8_t  diffT      = 22;    // |frame-bg| threshold (0..255)
  uint8_t  polarity   = 0;     // 0 abs, 1 hand brighter than bg, 2 darker
  uint8_t  orient     = 0;     // arm enters from 0 bottom,1 top,2 left,3 right
  uint8_t  mirror     = 1;     // flip horizontal cursor direction
  float    xmin       = 0.15f; // calibrated active range (fraction of canonical width)
  float    xmax       = 0.85f;
  uint16_t minArea    = 150;   // min hand area in pixels (of 19200)
  uint8_t  bandRows   = 30;    // rows from fingertip used for the cursor centroid
  uint8_t  fingerRows = 45;    // rows from fingertip scanned for fingers
  uint8_t  openRuns   = 3;     // >= this many fingers  -> open hand
  uint8_t  fistRuns   = 1;     // <= this many          -> fist
  uint8_t  nConfirm   = 3;     // consecutive frames to confirm a gesture change
  uint8_t  pendMax    = 12;    // max frames the cursor is frozen while ambiguous
  uint16_t lostMs     = 150;   // no hand this long -> tracking lost
  uint16_t lostRelMs  = 500;   // lost this long while grabbing -> forced release
  float    oeMinCut   = 1.0f;  // One-Euro min cutoff (Hz)
  float    oeBeta     = 0.0007f; // One-Euro speed coefficient (per unit/s, units 0..10000)
  uint8_t  roi[4]     = {0, 0, 100, 100}; // region of interest x0,y0,x1,y1 in % of raw image
  uint16_t magic      = 0xF176;
} P;
Preferences prefs;
struct HandObs { bool valid; float xn, yn; int fingers, area, tip, harea, sol1, sol2; };   // harea = hand-only area near the tip

// ---------------- One-Euro filter ----------------
struct OneEuro {
  bool init = false; float x = 0, dx = 0; uint32_t tPrev = 0;
  static float alpha(float cutoff, float dt) { float tau = 1.0f / (6.2831853f * cutoff); return 1.0f / (1.0f + tau / dt); }
  void reset(float v, uint32_t t) { init = true; x = v; dx = 0; tPrev = t; }
  float step(float v, uint32_t t) {
    if (!init) { reset(v, t); return v; }
    float dt = (t - tPrev) / 1000.0f; if (dt <= 0) dt = 0.001f; tPrev = t;
    float d = (v - x) / dt;
    dx += alpha(1.0f, dt) * (d - dx);
    float cut = P.oeMinCut + P.oeBeta * fabsf(dx);
    x += alpha(cut, dt) * (v - x);
    return x;
  }
} fx;

// ---------------- runtime state ----------------
bool verbose = false, dumpReq = false, grayReq = false, fullReq = false, studyLock = false;
bool engaged = false, tracked = false, haveOutput = false;
float outX = 5000, outY = 5000, offset = 0, prevFilt = 0, lastRaw = -1;
int goodStreak = 0, glitch = 0;
bool pending = false; float anchorX = 0; int pendFrames = 0, confirmCnt = 0;
float stableX = 5000;   // cursor while the fist was still solid, before the hand started opening
int prevSol = -1;
bool releaseHold = false;
uint32_t lastGoodMs = 0, fpsT0 = 0; int fpsN = 0; float fps = 0;
bool calRunning = false; uint32_t calEnd = 0; float calMin = 1, calMax = 0;
String cmdBuf;

// ---------------- helpers ----------------
static inline void canonDims() { if (P.orient <= 1) { CW = IW; CH = IH; } else { CW = IH; CH = IW; } }
static inline int canonIdx(int x, int y) {
  switch (P.orient) {
    case 0: return y * CW + x;                         // arm from bottom: r=y
    case 1: return (IH - 1 - y) * CW + (IW - 1 - x);   // arm from top
    case 2: return (IW - 1 - x) * CW + y;              // arm from left: r = W-1-x, c = y
    default: return x * CW + (IH - 1 - y);             // arm from right
  }
}
void ledEngaged(bool on) {
#if LED_PIN >= 0
  digitalWrite(LED_PIN, on ? LOW : HIGH);
#else
  (void)on;
#endif
}
static void serialTxTimeout(uint32_t ms) {
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(ms);
#else
  (void)ms;
#endif
}

void sendMove(uint32_t t) { Serial.printf("M,%lu,%d,%d\n", (unsigned long)t, (int)outX, (int)outY); }
void sendButton(uint32_t t, bool down, char why) { Serial.printf("B,%lu,%d,%c\n", (unsigned long)t, down ? 1 : 0, why); ledEngaged(down); }

void printParams() {
  Serial.printf("#P diffT=%u polarity=%u orient=%u mirror=%u xmin=%.3f xmax=%.3f minArea=%u bandRows=%u fingerRows=%u "
                "openRuns=%u fistRuns=%u nConfirm=%u pendMax=%u lostMs=%u lostRelMs=%u oeMinCut=%.3f oeBeta=%.5f\n",
                P.diffT, P.polarity, P.orient, P.mirror, P.xmin, P.xmax, P.minArea, P.bandRows, P.fingerRows,
                P.openRuns, P.fistRuns, P.nConfirm, P.pendMax, P.lostMs, P.lostRelMs, P.oeMinCut, P.oeBeta,
                P.roi[0], P.roi[1], P.roi[2], P.roi[3]);
}
void saveParams() { prefs.putBytes("p", &P, sizeof(P)); Serial.println("#saved"); }
void loadParams() {
  Params tmp;
  if (prefs.getBytesLength("p") == sizeof(Params)) { prefs.getBytes("p", &tmp, sizeof(tmp)); if (tmp.magic == 0xF176) P = tmp; }
  canonDims();
}

// ---------------- camera ----------------
static bool tryCamera(uint32_t xclk, int fbCount, camera_fb_location_t where, camera_grab_mode_t grab) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM; c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = xclk;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size = FRAMESIZE_QQVGA;      // 160 x 120
  c.jpeg_quality = 12;
  c.fb_count = fbCount;
  c.fb_location = where;
  c.grab_mode = grab;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("#camera init failed 0x%x xclk=%lu\n", err, (unsigned long)xclk);
    esp_camera_deinit();
    return false;
  }
  return true;
}

bool initCamera() {
  bool ok = false;
  if (psramFound()) ok = tryCamera(16000000, 2, CAMERA_FB_IN_PSRAM, CAMERA_GRAB_LATEST);
  if (!ok) ok = tryCamera(16000000, 1, CAMERA_FB_IN_DRAM, CAMERA_GRAB_WHEN_EMPTY);
  if (!ok) ok = tryCamera(10000000, 1, CAMERA_FB_IN_DRAM, CAMERA_GRAB_WHEN_EMPTY);
  if (!ok) return false;
  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("#camera PID=0x%x psram=%d\n", s->id.PID, psramFound() ? 1 : 0);
  if (s->id.PID == 0x3660 && s->set_vflip) s->set_vflip(s, 1);
  // Scale the whole sensor into 160x120 so a hand sweep stays inside the picture.
  if (s->set_res_raw) {
    int rc = s->set_res_raw(s, 0, 0, 2047, 1535, 0, 0, 2048, 1536, 160, 120, true, true);
    Serial.printf("#fov wide rc=%d\n", rc);
  }
  return true;
}

void setAutoExposure(bool on) {
  sensor_t *s = esp_camera_sensor_get();
  s->set_exposure_ctrl(s, on); s->set_gain_ctrl(s, on); s->set_whitebal(s, on);
  if (s->set_aec2) s->set_aec2(s, on);
}

void lockExposure() {
  setAutoExposure(true);
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) { camera_fb_t *fb = esp_camera_fb_get(); if (fb) esp_camera_fb_return(fb); }
  setAutoExposure(false);   // keep exposure/gain constant so background subtraction stays valid
  for (int i = 0; i < 4; i++) { camera_fb_t *fb = esp_camera_fb_get(); if (fb) esp_camera_fb_return(fb); }
  Serial.println("#exposure locked");
}

void captureBackground() {
  Serial.println("#capturing background - keep hand OUT of view");
  uint32_t *acc = (uint32_t *)heap_caps_calloc(NPIX, sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!acc) acc = (uint32_t *)calloc(NPIX, sizeof(uint32_t));
  if (!acc) { Serial.println("#no memory"); return; }
  const int K = 16; int got = 0;
  for (int k = 0; k < K + 4; k++) {
    camera_fb_t *fb = esp_camera_fb_get(); if (!fb) continue;
    if (k >= 4 && fb->len >= (size_t)NPIX) { for (int i = 0; i < NPIX; i++) acc[i] += fb->buf[i]; got++; }
    esp_camera_fb_return(fb);
  }
  if (got > 0) for (int i = 0; i < NPIX; i++) bg16[i] = (uint16_t)((acc[i] * 16) / got);
  free(acc);
  if (got == 0) { Serial.println("#background failed"); return; }
  Serial.printf("#background ok (%d frames)\n", got);
}

// ---------------- command parser ----------------
void handleCommand(const String &cmd) {
  if (cmd.length() == 0) return;
  char c = cmd[0]; long n = cmd.length() > 1 ? cmd.substring(1).toInt() : -1;
  switch (c) {
    case 'b': captureBackground(); break;
    case 'e': lockExposure(); captureBackground(); break;
    case 'c': calRunning = true; calEnd = millis() + 6000; calMin = 1; calMax = 0;
              Serial.println("#calibrating range: sweep open hand fully left and right for 6 s"); break;
    case 'o': if (n >= 0 && n <= 3) { P.orient = n; canonDims(); } Serial.printf("#orient=%u\n", P.orient); break;
    case 'm': P.mirror = !P.mirror; Serial.printf("#mirror=%u\n", P.mirror); break;
    case 't': if (n > 0 && n < 255) P.diffT = n; Serial.printf("#diffT=%u\n", P.diffT); break;
    case 'q': if (n >= 0 && n <= 2) P.polarity = n; Serial.printf("#polarity=%u\n", P.polarity); break;
    case 'a': if (n > 0) P.minArea = n; Serial.printf("#minArea=%u\n", P.minArea); break;
    case 'k': if (n > 0 && n <= 100) SOL_FIST = n; Serial.printf("#solFist=%d\n", SOL_FIST); break;
    case 'j': if (n > 0 && n <= 100) SOL_OPEN = n; Serial.printf("#solOpen=%d\n", SOL_OPEN); break;
    case 'u': if (n >= 1 && n < 10) MIN_RUN = n; Serial.printf("#minRun=%d\n", MIN_RUN); break;
    case 'x': if (n >= 0 && n < 10) P.fistRuns = n; Serial.printf("#fistRuns=%u (<= this = fist)\n", P.fistRuns); break;
    case 'y': if (n > 0 && n < 10) P.openRuns = n; Serial.printf("#openRuns=%u (>= this = open)\n", P.openRuns); break;
    case 'v': verbose = (n == 0 || n == 1) ? n : !verbose; break;
    case 'd': dumpReq = true; break;
    case 'g': grayReq = true; break;
    case 'G': fullReq = true; break;   // setup-only full snapshot for camera placement, never used by the app   // setup-only coarse preview, never used by the laptop app
    case 'p': printParams(); break;
    case 's': saveParams(); break;
    case 'R': { int v[4]; if (sscanf(cmd.c_str() + 1, "%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3]) == 4 &&
                    v[0] >= 0 && v[1] >= 0 && v[2] <= 100 && v[3] <= 100 && v[2] > v[0] + 10 && v[3] > v[1] + 10)
                  for (int k = 0; k < 4; k++) P.roi[k] = v[k];
                Serial.printf("#roi=%u,%u,%u,%u (send b to recapture background)\n", P.roi[0], P.roi[1], P.roi[2], P.roi[3]); break; }
    case 'z': if (n == 0 || n == 1) studyLock = (n == 1);
              if (studyLock) { verbose = false; dumpReq = false; grayReq = false; fullReq = false; }
              Serial.printf("#studyLock=%d\n", studyLock ? 1 : 0); break;
    case 'r': P = Params(); canonDims(); Serial.println("#defaults restored"); printParams(); break;
    default: Serial.printf("#unknown command '%s'\n", cmd.c_str());
  }
}
void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { cmdBuf.trim(); handleCommand(cmdBuf); cmdBuf = ""; }
    else if (cmdBuf.length() < 32) cmdBuf += ch;
  }
}

// ---------------- vision ----------------

HandObs processFrame(const uint8_t *img, bool adaptBg) {
  HandObs h = {false, 0, 0, 0, 0, -1};
  // 1) background subtraction into canonical mask; slow adaptation of non-foreground pixels
  const int T = P.diffT;
  const int rx0 = P.roi[0] * IW / 100, ry0 = P.roi[1] * IH / 100, rx1 = P.roi[2] * IW / 100, ry1 = P.roi[3] * IH / 100;
  for (int y = 0; y < IH; y++) {
    for (int x = 0; x < IW; x++) {
      int i = y * IW + x;
      int cur = img[i], b = bg16[i] >> 4, d = cur - b;
      bool fg = (P.polarity == 0) ? (d > T || -d > T) : (P.polarity == 1 ? d > T : -d > T);
      if (x < rx0 || x >= rx1 || y < ry0 || y >= ry1) fg = false;   // outside region of interest
      m1[canonIdx(x, y)] = fg;
      if (!fg && adaptBg) bg16[i] += ((cur << 4) - (int)bg16[i]) >> 5;
    }
  }
  // 2) 3x3 median (majority) filter: removes speckle, fills 1-px holes, keeps >=2-px finger gaps
  memset(m2, 0, CW * CH);
  for (int r = 1; r < CH - 1; r++) {
    const uint8_t *a = m1 + (r - 1) * CW, *b = m1 + r * CW, *c = m1 + (r + 1) * CW;
    for (int k = 1; k < CW - 1; k++) {
      int s = a[k-1] + a[k] + a[k+1] + b[k-1] + b[k] + b[k+1] + c[k-1] + c[k] + c[k+1];
      m2[r * CW + k] = s >= 5;
    }
  }
  // 3) largest 4-connected component
  const int N = CW * CH;
  memset(lab, 0, N * sizeof(int16_t));
  int16_t nextLab = 0, bestLab = 0; int bestArea = 0;
  for (int i = 0; i < N; i++) {
    if (!m2[i] || lab[i]) continue;
    if (nextLab == 32000) break;
    int16_t L = ++nextLab; int qh = 0, qt = 0, area = 0;
    lab[i] = L; bfsq[qt++] = i;
    while (qh < qt) {
      int p = bfsq[qh++]; area++;
      int r = p / CW, k = p - r * CW;
      if (k > 0      && m2[p-1]  && !lab[p-1])  { lab[p-1]  = L; bfsq[qt++] = p-1; }
      if (k < CW - 1 && m2[p+1]  && !lab[p+1])  { lab[p+1]  = L; bfsq[qt++] = p+1; }
      if (r > 0      && m2[p-CW] && !lab[p-CW]) { lab[p-CW] = L; bfsq[qt++] = p-CW; }
      if (r < CH - 1 && m2[p+CW] && !lab[p+CW]) { lab[p+CW] = L; bfsq[qt++] = p+CW; }
    }
    if (area > bestArea) { bestArea = area; bestLab = L; }
  }
  h.area = bestArea; gBestLab = bestLab;
  if (bestArea < P.minArea || bestArea > (N * 7) / 10) return h;   // nothing / global lighting change
  // 4) per-row statistics of the hand component (row 0 = far from the arm)
  int tip = -1;
  for (int r = 0; r < CH; r++) {
    const int16_t *L = lab + r * CW; int cnt = 0, runs = 0, runLen = 0, kmin = -1, kmax = -1; float sum = 0;
    for (int k = 0; k < CW; k++) {
      if (L[k] == bestLab) { cnt++; sum += k; runLen++; if (kmin < 0) kmin = k; kmax = k; }
      else { if (runLen >= MIN_RUN) runs++; runLen = 0; }
    }
    if (runLen >= MIN_RUN) runs++;
    rowCnt[r] = cnt; rowSum[r] = sum; rowRuns[r] = runs; rowSpan[r] = (kmin < 0) ? 0 : kmax - kmin + 1;
    if (tip < 0 && cnt >= 2) tip = r;
  }
  if (tip < 0) return h;
  h.tip = tip;
  // 5) cursor = centroid of the distal hand band
  float sx = 0, sy = 0; int n = 0;
  int rEnd = min(CH, tip + (int)P.bandRows);
  for (int r = tip; r < rEnd; r++) { sx += rowSum[r]; sy += (float)r * rowCnt[r]; n += rowCnt[r]; }
  if (n < 10) return h;
  h.xn = (sx / n) / (CW - 1); h.yn = (sy / n) / (CH - 1);
  // 6) finger count = max run count sustained over 3 consecutive rows near the tip
  int fEnd = min(CH - 2, tip + (int)P.fingerRows), best = 0;
  for (int r = tip; r < fEnd; r++) { int m = min(rowRuns[r], min(rowRuns[r+1], rowRuns[r+2])); if (m > best) best = m; }
  h.fingers = best;
  // hand-only area: pixels within fingerRows of the tip (excludes the forearm, which merges/splits)
  int ha = 0; for (int r = tip; r < min(CH, tip + (int)P.fingerRows); r++) ha += rowCnt[r];
  h.harea = ha;
  // solidity of the distal hand (filled px / row-span px): open hand with finger gaps -> low, fist -> ~1
  int sp2 = 0, a2 = 0;
  for (int r = tip; r < min(CH, tip + (int)P.fingerRows); r++) { sp2 += rowSpan[r]; a2 += rowCnt[r]; }
  h.sol2 = sp2 ? (100 * a2) / sp2 : 100;   // (debug) old measure: hand blob only
  // gesture solidity over ALL foreground near the hand: thin spread fingers often break off the
  // palm blob in the mask, so include fragments in a window that starts 35 rows ABOVE the blob tip
  // (where detached fingers are) and spans the hand's columns +-8 px.
  {
    int bx0 = CW, bx1 = -1;
    for (int r = tip; r < min(CH, tip + 25); r++)
      for (int k = 0; k < CW; k++) if (lab[r * CW + k] == bestLab) { if (k < bx0) bx0 = k; if (k > bx1) bx1 = k; }
    int x0 = max(0, bx0 - 8), x1 = min(CW - 1, bx1 + 8);
    int A = 0, S = 0;
    for (int r = max(0, tip - 35); r < min(CH, tip + 25); r++) {
      int c = 0, kmin = -1, kmax = -1;
      for (int k = x0; k <= x1; k++) if (m2[r * CW + k]) { c++; if (kmin < 0) kmin = k; kmax = k; }
      if (c) { A += c; S += kmax - kmin + 1; }
    }
    h.sol1 = S ? (100 * A) / S : 100;
  }
  h.valid = true;
  return h;
}

void dumpMask() {
  Serial.printf("#MASK %dx%d (2x down-sampled) '#'=hand '+'=other fg\n", CW, CH);
  char line[100];
  for (int r = 0; r < CH; r += 2) {
    int j = 0; line[j++] = '#'; line[j++] = ' ';
    for (int k = 0; k < CW && j < 98; k += 2) {
      int i = r * CW + k;
      line[j++] = m2[i] ? (lab[i] == gBestLab ? '#' : '+') : '.';
    }
    line[j] = 0; Serial.println(line);
  }
}

// ---------------- interaction logic ----------------
static uint32_t lastToggleMs = 0;   // time of the last grab/release (refractory period)
float mapX(float xn) {
  float u = (xn - P.xmin) / max(0.05f, P.xmax - P.xmin);
  if (P.mirror) u = 1.0f - u;
  return constrain(u, 0.0f, 1.0f) * 10000.0f;
}

void onInvalid(uint32_t t) {
  goodStreak = 0;
  if (tracked && t - lastGoodMs > P.lostMs) { tracked = false; pending = false; Serial.printf("T,%lu,0\n", (unsigned long)t); }
  if (engaged && t - lastGoodMs > P.lostRelMs) { engaged = false; sendMove(t); sendButton(t, false, 'l'); }
}

void onValid(const HandObs &h, uint32_t t) {
  float raw = mapX(h.xn);
  float rawY = constrain(h.yn, 0.0f, 1.0f) * 10000.0f;
  // reject single-frame outliers (another object, noise) that would cause cursor jumps
  if (tracked && lastRaw >= 0 && fabsf(raw - lastRaw) > 2500 && glitch < 3) { glitch++; onInvalid(t); return; }
  glitch = 0;
  if (!tracked) {                                  // re-acquisition: need a few stable frames first
    if (++goodStreak < 3) { lastRaw = raw; return; }
    tracked = true; fx.reset(raw, t); prevFilt = raw;
    offset = haveOutput ? (outX - raw) : 0;        // continue from where the cursor was -> no jump
    pending = false; confirmCnt = 0; prevSol = -1; releaseHold = false;
    Serial.printf("T,%lu,1\n", (unsigned long)t);
  }
  lastGoodMs = t; lastRaw = raw;
  float filt = fx.step(raw, t);
  float d = filt - prevFilt; prevFilt = filt;
  // Opening the fist moves the fingertip centroid before the open gesture is confirmed.
  // While a block is held, follow the hand only while the fist stays solid. A real open
  // drops solidity quickly; once that starts, keep the block where it was until the
  // hand is a tight fist again or the release is confirmed.
  if (!engaged) releaseHold = false;
  else if (releaseHold && h.sol1 >= SOL_FIST) releaseHold = false;   // fist tightened again: follow the drag
  else if (!releaseHold && prevSol >= 0 && (h.sol1 < SOL_HOLD || h.sol1 + 10 <= prevSol)) releaseHold = true;
  bool opening = engaged && releaseHold;
  prevSol = h.sol1;
  float cand;
  if (opening) {
    cand = stableX;
  } else {
    // reconcile any offset only while the hand moves (hidden in motion, no drift at rest)
    float shrink = 0.25f * fabsf(d);
    if (fabsf(offset) <= shrink) offset = 0; else offset -= (offset > 0 ? shrink : -shrink);
    cand = constrain(filt + offset, 0.0f, 10000.0f);
    if (engaged) stableX = cand;
  }
  outY += 0.3f * (rawY - outY);

  // gesture state machine with cursor freeze + roll-back (mitigates the "Heisenberg effect")
  // gesture class from the SOLIDITY of the distal hand (filled px / row-span px over 25 rows from
  // the tip): spread fingers leave gaps (measured ~50-60 %), a fist is a solid block (~99 %).
  // Mapped onto the run-count scale used by the state machine: 4 = open, 1 = fist, 2 = in between.
  // hysteresis: a grab must START with a tight fist (>= SOL_FIST), but while grabbing a fist that
  // relaxes to 85-91 % during the drag is still a fist -> no cursor freeze / release roll-back mid-drag
  int fistThr = engaged ? SOL_HOLD : SOL_FIST;
  int f = (h.sol1 >= fistThr) ? 1 : (h.sol1 <= SOL_OPEN ? 4 : 2);
  // a small blob (hand just entering the view, fingertips at the edge) is always "solid" ->
  // never let it start a grab; a real fist over the camera is several hundred px
  if (!engaged && h.harea < MIN_GRAB_AREA && f == 1) f = 2;
  // refractory period: no new toggle within 250 ms of the previous one (stops grab/release chatter)
  if (t - lastToggleMs < 250) f = engaged ? P.fistRuns : P.openRuns;
  bool wantChange = engaged ? (f >= P.openRuns) : (f <= P.fistRuns);
  bool ambiguous  = engaged ? (f > P.fistRuns)  : (f < P.openRuns);
  if (ambiguous) {
    if (!pending) { pending = true; anchorX = outX; pendFrames = 0; confirmCnt = 0; }
    pendFrames++;
    if (wantChange) confirmCnt++;
    if (confirmCnt >= P.nConfirm) {
      bool grabbing = !engaged;
      engaged = !engaged; pending = false; lastToggleMs = t;
      if (grabbing) stableX = anchorX;
      outX = grabbing ? anchorX : stableX;   // release where the fist was, not where the open hand drifted
      offset = outX - filt;
      sendMove(t); sendButton(t, engaged, 'g'); haveOutput = true;
      return;
    }
    if (pendFrames <= P.pendMax) return;           // frozen at anchor while the hand changes shape
    pending = false;                               // gave up: shape stayed ambiguous, resume
  } else {
    pending = false; confirmCnt = 0;
  }
  outX = cand; haveOutput = true;
  sendMove(t);
}

// ---------------- Arduino ----------------
void setup() {
#if LED_PIN >= 0
  pinMode(LED_PIN, OUTPUT); ledEngaged(false);
#endif
  Serial.begin(115200);
  serialTxTimeout(0);
  delay(1500);
  prefs.begin("fitts", false);
  loadParams();
  Serial.println("#FittsHandTracker boot");
  if (!initCamera()) { while (true) { delay(1000); Serial.println("#camera error"); } }
  lockExposure();
  captureBackground();
  printParams();
  Serial.println("#ready");
  fpsT0 = millis();
}

void loop() {
  pollSerial();
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { onInvalid(millis()); return; }
  uint32_t t = millis();
  if (fb->len < (size_t)NPIX) { esp_camera_fb_return(fb); return; }
  static uint8_t frameNo = 0; frameNo++;
  if (studyLock) { grayReq = false; fullReq = false; dumpReq = false; verbose = false; }
  if (grayReq) {                       // SETUP DIAGNOSTIC ONLY: 40x30 ASCII preview for camera placement
    grayReq = false; const char *ramp = " .:-=+*#%@";
    Serial.println("#GRAY 40x30 (raw camera orientation)");
    char line[48];
    for (int y = 0; y < IH; y += 4) { int j = 0; line[j++] = '#'; line[j++] = ' ';
      for (int x = 0; x < IW; x += 4) line[j++] = ramp[fb->buf[y * IW + x] * 10 / 256];
      line[j] = 0; Serial.println(line); }
  }
  if (fullReq) {                       // SETUP DIAGNOSTIC ONLY: one 160x120 frame as hex for placement checks
    fullReq = false; static const char hx[] = "0123456789abcdef"; char line[2 * IW + 4];
    serialTxTimeout(200);        // block while dumping so no line is dropped
    Serial.printf("#IMG %d %d\n", IW, IH);
    for (int y = 0; y < IH; y++) { int j = 0; line[j++] = '#'; line[j++] = 'I';
      for (int x = 0; x < IW; x++) { uint8_t v = fb->buf[y * IW + x]; line[j++] = hx[v >> 4]; line[j++] = hx[v & 15]; }
      line[j] = 0; Serial.println(line); }
    Serial.flush(); serialTxTimeout(0);
  }
  HandObs h = processFrame(fb->buf, (frameNo & 3) == 0);
  esp_camera_fb_return(fb);

  if (calRunning) {
    if (h.valid) { calMin = min(calMin, h.xn); calMax = max(calMax, h.xn); }
    if (millis() > calEnd) {
      calRunning = false;
      if (calMax - calMin > 0.15f) { P.xmin = calMin + 0.03f; P.xmax = calMax - 0.03f; Serial.printf("#range xmin=%.3f xmax=%.3f (send 's' to save)\n", P.xmin, P.xmax); }
      else Serial.println("#calibration failed: range too small");
    }
  }

  if (h.valid) onValid(h, t); else onInvalid(t);

  fpsN++;
  if (t - fpsT0 >= 1000) { fps = fpsN * 1000.0f / (t - fpsT0); fpsN = 0; fpsT0 = t; }
  if (verbose) Serial.printf("#V v=%d f=%d area=%d tip=%d xn=%.3f out=%d eng=%d pend=%d fps=%.1f ha=%d s1=%d s2=%d\n",
                             h.valid, h.fingers, h.area, h.tip, h.xn, (int)outX, engaged, pending, fps, h.harea, h.sol1, h.sol2);
  if (dumpReq) { dumpReq = false; dumpMask(); }
}
