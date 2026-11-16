"""Live trial tracker. Re-reads data/*.csv every time the page asks.

  python analysis/track.py

Practice rows are listed in the table and are not drawn on the plot.
The plot is successful recorded trials only: ID on x, movement time on y.
"""
from __future__ import annotations

import csv
import json
import math
import os
import webbrowser
from collections import defaultdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")
PORT = 8765
WIDTHS = (120, 240, 480)
DISTANCES = (480, 1080, 1920)
NEED = 10
COLORS = ["#2a6fdb", "#e0473a", "#2e9e5b", "#9b5de5", "#f08c00", "#0fa3b1"]


def planned_keys():
    return [(d, w) for w in WIDTHS for d in DISTANCES]


def load_rows():
    rows = []
    if not os.path.isdir(DATA):
        return rows
    for name in sorted(os.listdir(DATA)):
        if not name.endswith(".csv"):
            continue
        path = os.path.join(DATA, name)
        with open(path, encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                if row.get("input") == "mouse" or not row.get("participant"):
                    continue
                row["_file"] = name
                rows.append(row)
    return rows


def num(row, key, cast=float):
    try:
        return cast(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def payload():
    rows = load_rows()
    by_pid = defaultdict(list)
    for row in rows:
        by_pid[row["participant"]].append(row)
    participants = []
    points = []
    for pid in sorted(by_pid):
        group = by_pid[pid]
        recorded = [r for r in group if r.get("phase") == "recorded"]
        success = defaultdict(int)
        fail = defaultdict(int)
        for row in recorded:
            d, w = num(row, "D_px", int), num(row, "W_px", int)
            if d is None or w is None:
                continue
            if row.get("success") == "1":
                success[(d, w)] += 1
                points.append({
                    "participant": pid,
                    "id": num(row, "ID_bits"),
                    "mt": num(row, "MT_ms"),
                })
            else:
                fail[(d, w)] += 1
        keys = planned_keys()
        for key in sorted(set(success) | set(fail)):
            if key not in keys:
                keys.append(key)
        conditions = []
        for d, w in keys:
            conditions.append({
                "D": d,
                "W": w,
                "ID": round(math.log2(d / w + 1), 3),
                "success": success[(d, w)],
                "fail": fail[(d, w)],
                "need": NEED,
                "done": success[(d, w)] >= NEED,
            })
        participants.append({
            "id": pid,
            "recorded_success": sum(success.values()),
            "recorded_fail": sum(fail.values()),
            "practice": sum(1 for r in group if r.get("phase") == "practice"),
            "goal": 9 * NEED,
            "conditions": conditions,
        })
    trials = []
    for row in rows:
        trials.append({
            "file": row["_file"],
            "participant": row.get("participant", ""),
            "phase": row.get("phase", ""),
            "attempt": row.get("attempt", ""),
            "D": row.get("D_px", ""),
            "W": row.get("W_px", ""),
            "ID": row.get("ID_bits", ""),
            "MT": row.get("MT_ms", ""),
            "success": row.get("success", ""),
            "overlap": row.get("overlap", ""),
            "reason": row.get("release_reason", ""),
        })
    return {
        "participants": participants,
        "trials": trials,
        "points": [p for p in points if p["id"] is not None and p["mt"] is not None],
        "colors": COLORS,
        "needed_people": 3,
    }


PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Fitts trial tracker</title>
<style>
  body { margin: 0; font: 15px/1.45 Segoe UI, sans-serif; background: #f4f5f7; color: #1c1f26; }
  header { padding: 22px 28px 8px; }
  h1 { font-size: 22px; margin: 0 0 4px; }
  p.note { margin: 0; color: #5c6370; }
  main { padding: 8px 28px 40px; }
  .cards { display: flex; flex-wrap: wrap; gap: 14px; margin: 16px 0; }
  .card { background: #fff; border: 1px solid #e1e4ea; border-radius: 8px; padding: 14px 16px; min-width: 280px; }
  .card h2 { margin: 0 0 6px; font-size: 16px; }
  table { border-collapse: collapse; width: 100%; background: #fff; }
  th, td { border-bottom: 1px solid #e6e8ee; text-align: left; padding: 6px 8px; font-variant-numeric: tabular-nums; }
  th { font-size: 12px; color: #5c6370; font-weight: 600; }
  .ok { color: #1b7f45; } .bad { color: #b42318; }
  .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; margin-top: 8px; }
  .cell { border: 1px solid #e1e4ea; border-radius: 6px; padding: 6px 8px; font-size: 13px; }
  .cell.done { background: #e8f6ee; border-color: #b7e0c6; }
  svg { background: #fff; border: 1px solid #e1e4ea; border-radius: 8px; }
  .legend { display: flex; gap: 14px; margin: 8px 0 0; font-size: 13px; }
  .swatch { display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin-right: 6px; }
</style>
</head>
<body>
<header>
  <h1>Fitts trial tracker</h1>
  <p class="note">This page re-reads the logs every few seconds. The plot uses successful recorded trials only. Practice stays in the table and off the plot.</p>
</header>
<main>
  <section id="progress"></section>
  <h2>Index of difficulty and movement time</h2>
  <div id="plot"></div>
  <h2>Trials</h2>
  <table>
    <thead>
      <tr><th>Participant</th><th>Phase</th><th>Attempt</th><th>D (px)</th><th>W (px)</th><th>ID (bits)</th><th>Movement time (ms)</th><th>Success</th><th>Overlap</th><th>Log</th></tr>
    </thead>
    <tbody id="rows"></tbody>
  </table>
</main>
<script>
const COLORS = ["#2a6fdb", "#e0473a", "#2e9e5b", "#9b5de5", "#f08c00", "#0fa3b1"];
function color(i) { return COLORS[i % COLORS.length]; }
function render(data) {
  const progress = document.getElementById("progress");
  const people = data.participants;
  if (!people.length) {
    progress.innerHTML = "<p>No logs yet. Start a session and this page will fill in.</p>";
  } else {
    const missing = Math.max(0, data.needed_people - people.length);
    const cards = people.map(p => {
      const cells = p.conditions.map(c =>
        `<div class="cell ${c.done ? "done" : ""}">W ${c.W} · D ${c.D}<br>ID ${c.ID} bits<br>${c.success}/${c.need} successes${c.fail ? ` · ${c.fail} misses` : ""}</div>`
      ).join("");
      return `<article class="card"><h2>${p.id}</h2><div>${p.recorded_success}/${p.goal} recorded successes · ${p.recorded_fail} misses · ${p.practice} practice</div><div class="grid">${cells}</div></article>`;
    }).join("");
    const note = missing ? `<p class="note">The assignment still needs ${missing} more participant${missing === 1 ? "" : "s"}.</p>` : "";
    progress.className = "";
    progress.innerHTML = `<div class="cards">${cards}</div>${note}`;
  }
  draw(data);
  const body = document.getElementById("rows");
  const trials = data.trials.slice().reverse();
  body.innerHTML = trials.map(t => {
    const hit = t.success === "1";
    return `<tr><td>${t.participant}</td><td>${t.phase}</td><td>${t.attempt}</td><td>${t.D}</td><td>${t.W}</td><td>${t.ID}</td><td>${t.MT}</td><td class="${hit ? "ok" : "bad"}">${hit ? "success" : "miss"}</td><td>${t.overlap}</td><td>${t.file}</td></tr>`;
  }).join("") || "<tr><td colspan='10'>No attempts yet.</td></tr>";
}
function draw(data) {
  const host = document.getElementById("plot");
  const W = 760, H = 460, L = 64, R = 20, T = 20, B = 48;
  const pts = data.points;
  const pids = [...new Set(pts.map(p => p.participant))];
  const maxId = Math.max(4.5, ...pts.map(p => p.id));
  const maxMt = Math.max(1000, ...pts.map(p => p.mt));
  const x = v => L + (v / (maxId * 1.08)) * (W - L - R);
  const y = v => H - B - (v / (maxMt * 1.08)) * (H - T - B);
  let ticks = "";
  for (let i = 0; i <= 4; i++) {
    const id = (maxId * i) / 4, mt = (maxMt * i) / 4;
    ticks += `<line x1="${x(id)}" y1="${H-B}" x2="${x(id)}" y2="${T}" stroke="#e6e8ee"/>`;
    ticks += `<text x="${x(id)}" y="${H-B+18}" text-anchor="middle" font-size="12" fill="#5c6370">${id.toFixed(1)}</text>`;
    ticks += `<line x1="${L}" y1="${y(mt)}" x2="${W-R}" y2="${y(mt)}" stroke="#e6e8ee"/>`;
    ticks += `<text x="${L-8}" y="${y(mt)+4}" text-anchor="end" font-size="12" fill="#5c6370">${Math.round(mt)}</text>`;
  }
  const dots = pts.map(p => `<circle cx="${x(p.id)}" cy="${y(p.mt)}" r="4" fill="${color(pids.indexOf(p.participant))}"><title>${p.participant}: ID ${p.id.toFixed(2)} bits, ${Math.round(p.mt)} ms</title></circle>`).join("");
  const empty = pts.length ? "" : `<text x="${(L+W-R)/2}" y="${(T+H-B)/2}" text-anchor="middle" fill="#5c6370">No successful recorded trials yet</text>`;
  host.innerHTML = `<svg width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">
    ${ticks}
    <line x1="${L}" y1="${H-B}" x2="${W-R}" y2="${H-B}" stroke="#1c1f26"/>
    <line x1="${L}" y1="${T}" x2="${L}" y2="${H-B}" stroke="#1c1f26"/>
    <text x="${(L+W-R)/2}" y="${H-8}" text-anchor="middle" font-size="13">Index of difficulty (bits)</text>
    <text x="16" y="${(T+H-B)/2}" text-anchor="middle" font-size="13" transform="rotate(-90 16 ${(T+H-B)/2})">Movement time (ms)</text>
    ${dots}${empty}
  </svg>
  <div class="legend">${pids.map((id, i) => `<span><i class="swatch" style="background:${color(i)}"></i>${id}</span>`).join("") || "<span>Legend appears when a recorded success is logged.</span>"}</div>`;
}
async function refresh() {
  const res = await fetch("/api/trials");
  render(await res.json());
}
refresh();
setInterval(refresh, 3000);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/api/trials"):
            body = json.dumps(payload()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        body = PAGE.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        return


def main():
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    webbrowser.open(f"http://127.0.0.1:{PORT}/")
    print(f"tracker http://127.0.0.1:{PORT}/")
    server.serve_forever()


if __name__ == "__main__":
    main()
