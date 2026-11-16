"""Analyse Fitts' Law study logs.

  python analysis/analyze.py            # reads data/*.csv, writes analysis/out/*

Outputs:
  fitts_all_trials.png   every successful recorded trial, MT vs ID, one colour per participant (+ fit lines)
  fitts_means.png        per-participant mean MT per condition with regression lines
  summary.txt            regression (a, b, R^2), throughput, unsuccessful-attempt counts
  per_condition.csv      mean / sd MT and failures per participant x condition
"""
import csv
import glob
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "analysis", "out")
COLORS = ["#2a6fdb", "#e0473a", "#2e9e5b", "#9b5de5", "#f08c00", "#0fa3b1"]


def load():
    rows = []
    for p in sorted(glob.glob(os.path.join(DATA, "*.csv"))):
        with open(p, encoding="utf-8") as f:
            for r in csv.DictReader(f):
                if r.get("input") == "mouse":
                    continue  # debug sessions are never analysed
                rows.append(r)
    return rows


def fit(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    b, a = np.polyfit(x, y, 1)
    yhat = a + b * x
    r2 = 1 - np.sum((y - yhat) ** 2) / np.sum((y - y.mean()) ** 2)
    return a, b, r2


def main():
    os.makedirs(OUT, exist_ok=True)
    rows = load()
    rec = [r for r in rows if r["phase"] == "recorded"]
    pids = sorted({r["participant"] for r in rec})
    succ = defaultdict(list); fails = defaultdict(lambda: defaultdict(int)); attempts = defaultdict(int)
    for r in rec:
        key = (int(r["D_px"]), int(r["W_px"]))
        attempts[r["participant"]] += 1
        if r["success"] == "1":
            succ[r["participant"]].append((float(r["ID_bits"]), float(r["MT_ms"]), key, r))
        else:
            fails[r["participant"]][key] += 1

    lines = ["Fitts' Law analysis (successful recorded trials only)", ""]
    # ---- figure 1: all trials ----
    fig, ax = plt.subplots(figsize=(8, 5.2), dpi=150)
    for i, p in enumerate(pids):
        c = COLORS[i % len(COLORS)]
        ids = [s[0] for s in succ[p]]; mts = [s[1] for s in succ[p]]
        jitter = np.random.default_rng(i).uniform(-0.03, 0.03, len(ids))
        ax.scatter(np.array(ids) + jitter, mts, s=14, alpha=0.45, color=c, edgecolors="none")
        a, b, r2 = fit(ids, mts)
        xs = np.linspace(0, max(ids) + 0.3, 50)
        ax.plot(xs, a + b * xs, color=c, lw=2, label=f"{p}: MT = {a:.0f} + {b:.0f}·ID  (R²={r2:.2f})")
    ax.set_xlabel("Index of Difficulty, ID = log₂(D/W + 1)  [bits]")
    ax.set_ylabel("Movement Time, MT  [ms]")
    ax.set_title("Fitts' Law: all successful trials (ESP32 camera hand interface)")
    ax.set_xlim(0, None); ax.set_ylim(0, None); ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fitts_all_trials.png")); plt.close(fig)

    # ---- figure 2: condition means ----
    fig, ax = plt.subplots(figsize=(8, 5.2), dpi=150)
    pc_rows = []
    for i, p in enumerate(pids):
        c = COLORS[i % len(COLORS)]
        by = defaultdict(list)
        for idb, mt, key, _ in succ[p]:
            by[key].append(mt)
        keys = sorted(by, key=lambda k: math.log2(k[0] / k[1] + 1))
        mids = [math.log2(k[0] / k[1] + 1) for k in keys]; mmeans = [np.mean(by[k]) for k in keys]
        a, b, r2 = fit(mids, mmeans)
        ax.plot(mids, mmeans, "o", color=c, ms=6)
        xs = np.linspace(0, max(mids) + 0.3, 50)
        ax.plot(xs, a + b * xs, ":", color=c, lw=2, label=f"{p}: a = {a:.0f} ms, b = {b:.0f} ms/bit (R²={r2:.2f})")
        tp = np.mean([math.log2(k[0] / k[1] + 1) / (np.mean(by[k]) / 1000) for k in keys])
        lines.append(f"{p}: n_success={len(succ[p])}  attempts={attempts[p]}  unsuccessful={sum(fails[p].values())}")
        lines.append(f"    trial-level fit : MT = {fit([s[0] for s in succ[p]], [s[1] for s in succ[p]])[0]:.1f} + "
                     f"{fit([s[0] for s in succ[p]], [s[1] for s in succ[p]])[1]:.1f}*ID   R2={fit([s[0] for s in succ[p]], [s[1] for s in succ[p]])[2]:.3f}")
        lines.append(f"    mean-level fit  : MT = {a:.1f} + {b:.1f}*ID   R2={r2:.3f}   mean throughput={tp:.2f} bits/s")
        for k in keys:
            pc_rows.append(dict(participant=p, D_px=k[0], W_px=k[1], ID_bits=round(math.log2(k[0] / k[1] + 1), 3),
                                n_success=len(by[k]), MT_mean_ms=round(float(np.mean(by[k])), 1),
                                MT_sd_ms=round(float(np.std(by[k], ddof=1)), 1) if len(by[k]) > 1 else "",
                                unsuccessful=fails[p][k]))
    ax.set_xlabel("Index of Difficulty, ID = log₂(D/W + 1)  [bits]")
    ax.set_ylabel("Mean Movement Time, MT  [ms]")
    ax.set_title("Fitts' Law: mean MT per condition")
    ax.set_xlim(0, None); ax.set_ylim(0, None); ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fitts_means.png")); plt.close(fig)

    if succ:
        allx = [s[0] for p in pids for s in succ[p]]; ally = [s[1] for p in pids for s in succ[p]]
        a, b, r2 = fit(allx, ally)
        lines += ["", f"Pooled (all participants, trial level): MT = {a:.1f} + {b:.1f}*ID  R2={r2:.3f}"]
    lines += ["", "Unsuccessful attempts per participant x (D, W):"]
    for p in pids:
        lines.append(f"  {p}: " + ", ".join(f"D{d}/W{w}={n}" for (d, w), n in sorted(fails[p].items()) if n) +
                     f"   total={sum(fails[p].values())}")
    rel = [r for r in rec if r["release_reason"] == "l"]
    lines.append(f"\nReleases forced by tracking loss: {len(rel)}")
    sk = [r for r in rec if r["release_reason"] == "skip"]
    lines.append(f"Attempts skipped by the operator (K, counted as unsuccessful): {len(sk)}")

    with open(os.path.join(OUT, "per_condition.csv"), "w", newline="", encoding="utf-8") as f:
        if pc_rows:
            w = csv.DictWriter(f, fieldnames=list(pc_rows[0])); w.writeheader(); w.writerows(pc_rows)
    with open(os.path.join(OUT, "summary.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
