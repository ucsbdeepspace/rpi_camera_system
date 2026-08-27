#!/usr/bin/env python3
"""
Plots the corrected-notch Ki sweep (2026-08-27) alongside the earlier
no-notch and buggy-notch (mis-clocked ~16.8Hz) sweeps, for the notch
sample-rate bug documented in CLAUDE.md ("Found and fixed: the notch
filter's sample-rate bug was still live in firmware").

cmd_set_notch previously hardcoded notch_configure()'s sample rate to
457.5Hz regardless of the real control rate -- at throttled 200Hz (the
rate every one of these trials uses), that put a notch REQUESTED at
38.5Hz actually at ~16.8Hz, carving a hole in the 10-20Hz band instead of
suppressing the real resonance. Fixed by passing the real
g_ctrl_rate_millihz through instead. This script compares the Ki
stability ceiling across all three conditions:
  - no notch (unaffected by the bug -- notch_configure is never called)
  - buggy notch (~16.8Hz, mis-clocked)
  - real notch (38.5Hz, post-fix)

All trials: Kp=1.75 counts/px, throttled to 200Hz, -25px step at
dac_y=2048, via scratch_notch_comparison.py's run_trial (explicit
set_mode closed_loop confirmation, not just a trusted reply -- see that
script's own history for why that mattered).

Usage: python3 fta_notch_ki_sweep_plot.py
Output:
  results/fta_notch_sweep_panel.png          -- 6-panel small multiples,
                                                 real-notch trials only
  results/fta_notch_ki_ceiling_comparison.png -- overshoot vs Ki, all
                                                 three conditions overlaid
"""
import glob
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from fta_closed_loop_step_response_vcp import analyze_step

BLUE = "#2a78d6"
ORANGE = "#eb6834"
RED = "#c0392b"
GREEN = "#3a8a4a"
TARGET_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"

# (Ki, path, verdict-from-the-bench-session) -- verdict is what
# scratch_notch_comparison.py's own amplitude-based classifier reported
# live, kept here rather than re-derived so the panel matches exactly
# what was reported at the time.
REAL_NOTCH_RUNS = [
    (200, "results/scratch_notch_cmp_notchfix_ki200_20260827T135906.npz", "CLEAN"),
    (300, "results/scratch_notch_cmp_notchfix_ki300_20260827T140134.npz", "CLEAN"),
    (350, "results/scratch_notch_cmp_notchfix_ki350_20260827T140204.npz", "CLEAN"),
    (400, "results/scratch_notch_cmp_notchfix_ki400_20260827T135944.npz", "RINGING"),
    (550, "results/scratch_notch_cmp_notchfix_ki550_20260827T140027.npz", "RINGING"),
    (800, "results/scratch_notch_cmp_notchfix_ki800_20260827T140102.npz", "DIVERGED"),
]

NO_NOTCH_GLOB = "results/scratch_notch_cmp_nonotch_ki*.npz"
NO_NOTCH_BASELINE = "results/scratch_notch_cmp_baseline200_20260819T113307.npz"  # Ki=200, no notch
BUGGY_NOTCH_GLOB = "results/scratch_notch_cmp_notch_ki*.npz"

KI_RE = re.compile(r"_ki(\d+)_")


def style_axis(ax):
    ax.set_facecolor("white")
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8.5, length=3)


def load_overshoot(path):
    d = np.load(path)
    t, x, t_step = d["t"], d["x"], float(d["t_step"])
    metrics = analyze_step(t, x, t_step, 2.0)
    max_dev = float(np.max(np.abs(x - float(d["target_from"]))))
    step_px = abs(float(d["target_to"]) - float(d["target_from"]))
    if metrics is None or metrics["overshoot_pct"] is None:
        # Diverged badly enough that analyze_step's own settle/overshoot
        # math breaks down -- report a floor estimate from raw excursion
        # instead of dropping the point.
        return max(0.0, (max_dev / step_px - 1.0) * 100.0) if step_px > 0 else None
    return metrics["overshoot_pct"]


def panel_plot():
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.5), dpi=180)
    fig.patch.set_facecolor("white")

    for ax, (ki, path, verdict) in zip(axes.flat, REAL_NOTCH_RUNS):
        d = np.load(path)
        t, x, t_step = d["t"], d["x"], float(d["t_step"])
        target_from, target_to = int(d["target_from"]), int(d["target_to"])

        style_axis(ax)
        ax.axhline(target_from, color=TARGET_COLOR, linewidth=1.0, linestyle=(0, (2, 2)), alpha=0.7)
        ax.plot([t_step, t[-1]], [target_to, target_to], color=TARGET_COLOR,
                linewidth=1.1, linestyle=(0, (2, 2)))

        color = {"CLEAN": BLUE, "RINGING": ORANGE, "DIVERGED": RED}[verdict]
        ax.plot(t, x, color=color, linewidth=1.2)
        ax.axvline(t_step, color=MUTED, linewidth=0.7, linestyle=(0, (1, 2)))

        ax.set_title(f"Ki={ki}  ({verdict})", fontsize=11, color="#1A1A2E",
                     fontweight="bold" if verdict != "CLEAN" else "normal", loc="left")
        ax.set_xlabel("time (s)", fontsize=8.5, color=MUTED)
        ax.set_ylabel("cx (px)", fontsize=8.5, color=MUTED)

    fig.suptitle("Corrected notch (38.5Hz, post sample-rate-bug fix) -- Ki sweep, Kp=1.75, throttled 200Hz",
                 fontsize=13, color="#1A1A2E", x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig("results/fta_notch_sweep_panel.png", facecolor="white")
    print("Saved results/fta_notch_sweep_panel.png")


def comparison_plot():
    series = {"no notch": {}, "buggy notch (~16.8Hz)": {}, "real notch (38.5Hz)": {}}

    for path in glob.glob(NO_NOTCH_GLOB):
        m = KI_RE.search(path)
        if m:
            series["no notch"][int(m.group(1))] = load_overshoot(path)
    series["no notch"][200] = load_overshoot(NO_NOTCH_BASELINE)

    for path in glob.glob(BUGGY_NOTCH_GLOB):
        m = KI_RE.search(path)
        if m:
            series["buggy notch (~16.8Hz)"][int(m.group(1))] = load_overshoot(path)

    for ki, path, _ in REAL_NOTCH_RUNS:
        series["real notch (38.5Hz)"][ki] = load_overshoot(path)

    fig, ax = plt.subplots(figsize=(9, 6), dpi=150)
    style_axis(ax)
    ax.set_facecolor("white")

    colors = {"no notch": MUTED, "buggy notch (~16.8Hz)": ORANGE, "real notch (38.5Hz)": BLUE}
    markers = {"no notch": "o", "buggy notch (~16.8Hz)": "s", "real notch (38.5Hz)": "^"}

    for label, data in series.items():
        if not data:
            continue
        kis = sorted(data.keys())
        vals = [data[k] for k in kis]
        ax.plot(kis, vals, color=colors[label], marker=markers[label], linewidth=1.8,
                markersize=7, label=label)

    ax.axhline(25, color=RED, linewidth=0.8, linestyle=(0, (2, 2)), alpha=0.6)
    ax.text(ax.get_xlim()[1] if ax.get_xlim()[1] > 0 else 800, 27,
            "ringing threshold (25%)", fontsize=8, color=RED, ha="right")

    ax.set_xlabel("Ki (counts / px / s)", fontsize=10, color=MUTED)
    ax.set_ylabel("overshoot (%)", fontsize=10, color=MUTED)
    ax.set_title("Ki stability ceiling: real notch vs. buggy notch vs. no notch\n"
                 "(Kp=1.75, throttled 200Hz, -25px step)", fontsize=12.5, color="#1A1A2E", loc="left")
    ax.legend(frameon=False, fontsize=9.5, loc="upper left")
    ax.set_yscale("symlog", linthresh=50)

    fig.tight_layout()
    fig.savefig("results/fta_notch_ki_ceiling_comparison.png", facecolor="white")
    print("Saved results/fta_notch_ki_ceiling_comparison.png")


if __name__ == "__main__":
    panel_plot()
    comparison_plot()
