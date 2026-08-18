#!/usr/bin/env python3
"""
Combines the 2026-08-13 on-board-sine-generator closed-loop tracking runs
(1/5/10/15/20Hz) into one summary figure: per-frequency traces plus
gain/lag vs. frequency, the first real closed-loop frequency-response
characterization across the project's actual 10-20Hz disturbance target.

results/ can hold runs at more than one commanded amplitude (e.g. the
original 25px/150um-peak-to-peak sweep and a later 1.667px/10um-peak-to-
peak sweep) -- selection is by each npz's own stored amplitude_px, not
just "whichever file for this frequency happens to glob last", so a
second sweep at a different amplitude can't silently clobber which run
gets plotted.

Usage: python3 fta_closed_loop_onboard_sine_plot.py [--amplitude-px PX] [--out PATH]
  --amplitude-px: select the sweep whose commanded amplitude is closest
                   to this value (default: the largest amplitude found,
                   i.e. the original 25px sweep, for backward compat)
Output: results/fta_closed_loop_onboard_sine_summary.png (or --out)
"""
import argparse
import glob
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE = "#2a78d6"
ORANGE = "#eb6834"
TARGET_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"

FREQS = [1, 5, 10, 15, 20]


def find_files():
    """Returns {freq: [(amplitude_px, path), ...]}, amplitude read from
    each npz itself (not guessed from the filename)."""
    by_freq = {}
    for f in glob.glob("results/fta_closed_loop_onboard_sine_*Hz_*.npz"):
        m = re.search(r"onboard_sine_(\d+(?:\.\d+)?)Hz_", f)
        if not m:
            continue
        freq = float(m.group(1))
        amplitude_px = float(np.load(f)["amplitude_px"])
        by_freq.setdefault(freq, []).append((amplitude_px, f))
    return by_freq


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--amplitude-px", type=float, default=None,
                         help="pick the sweep closest to this commanded amplitude; default: largest available")
    parser.add_argument("--out", default="results/fta_closed_loop_onboard_sine_summary.png")
    args = parser.parse_args()

    by_freq = find_files()
    runs = []
    for freq in FREQS:
        candidates = by_freq.get(float(freq))
        if not candidates:
            print(f"missing data for {freq}Hz, skipping")
            continue
        if args.amplitude_px is None:
            amplitude_px, path = max(candidates, key=lambda c: c[0])
        else:
            amplitude_px, path = min(candidates, key=lambda c: abs(c[0] - args.amplitude_px))
        d = np.load(path)
        # dac_y wasn't recorded before 2026-08-14 -- older npz files won't
        # have it; fall back to NaN (renders as a gap, not a misleading
        # flat line) rather than erroring out on an older sweep.
        dac_y = d["dac_y"] if "dac_y" in d.files else np.full_like(d["t"], np.nan)
        runs.append({
            "freq": freq, "t": d["t"], "x": d["x"], "tgt": d["tgt"], "dac_y": dac_y,
            "center_px": float(d["center_px"]), "amplitude_px": float(d["amplitude_px"]),
            "gain": float(d["gain"]), "lag_ms": float(d["lag_ms"]),
        })
    if not runs:
        print("No matching data found.")
        return
    amplitude_used = runs[0]["amplitude_px"]
    um = 3.0
    print(f"Using amplitude={amplitude_used:.2f}px ({amplitude_used*2*um:.1f}um peak-to-peak) across {len(runs)} runs")

    fig = plt.figure(figsize=(16, 10.5), dpi=180)
    fig.patch.set_facecolor("white")
    gs = fig.add_gridspec(3, len(runs), height_ratios=[1.3, 0.8, 1], hspace=0.55, wspace=0.35)

    for col, run in enumerate(runs):
        ax = fig.add_subplot(gs[0, col])
        ax_dac = fig.add_subplot(gs[1, col], sharex=ax)
        for a in (ax, ax_dac):
            a.set_facecolor("white")
            for spine in ("top", "right"):
                a.spines[spine].set_visible(False)
            for spine in ("left", "bottom"):
                a.spines[spine].set_color(GRID)
            a.tick_params(colors=MUTED, labelsize=8, length=3)

        t, x, tgt, dac_y = run["t"], run["x"], run["tgt"], run["dac_y"]
        ax.plot(t, tgt * um, color=TARGET_COLOR, linewidth=1.0, linestyle=(0, (2, 2)))
        ax.plot(t, x * um, color=BLUE, linewidth=1.1)
        ax.set_title(f"{run['freq']}Hz", fontsize=11, fontweight="bold", color="#0b0b0b")
        plt.setp(ax.get_xticklabels(), visible=False)
        if col == 0:
            ax.set_ylabel("cx (µm)", fontsize=8.5, color=MUTED)
        if col == len(runs) - 1:
            sec = ax.secondary_yaxis("right", functions=(lambda v: v / um, lambda px: px * um))
            sec.tick_params(colors=MUTED, labelsize=7.5, length=3)
            sec.set_ylabel("px", fontsize=8, color=MUTED)
        ax.text(0.03, 0.03, f"gain {run['gain']:.2f}\nlag {run['lag_ms']:.0f}ms",
                transform=ax.transAxes, fontsize=7.5, color="#0b0b0b", va="bottom", ha="left",
                bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=2))

        ax_dac.plot(t, dac_y, color=ORANGE, linewidth=1.0)
        ax_dac.set_xlabel("time (s)", fontsize=8, color=MUTED)
        if col == 0:
            ax_dac.set_ylabel("dac_y (counts)", fontsize=8, color=MUTED)

    ax_gain = fig.add_subplot(gs[2, 0:len(runs)//2 + 1])
    ax_lag = fig.add_subplot(gs[2, len(runs)//2 + 1:])
    for ax in (ax_gain, ax_lag):
        ax.set_facecolor("white")
        ax.grid(True, color=GRID, linewidth=0.6)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)

    freqs = [r["freq"] for r in runs]
    gains = [r["gain"] for r in runs]
    lags = [r["lag_ms"] for r in runs]

    ax_gain.plot(freqs, gains, color=BLUE, linewidth=2, marker="o", markersize=6)
    ax_gain.axhline(1.0, color=MUTED, linewidth=0.8, linestyle=(0, (2, 2)))
    ax_gain.set_xlabel("frequency (Hz)", fontsize=9, color=MUTED)
    ax_gain.set_ylabel("tracking gain (T)", fontsize=9, color=MUTED)
    ax_gain.set_title("Closed-loop gain vs. frequency", fontsize=10.5, fontweight="bold")
    ax_gain.set_xticks(freqs)

    ax_lag.plot(freqs, lags, color=ORANGE, linewidth=2, marker="o", markersize=6)
    ax_lag.axhline(0, color=MUTED, linewidth=0.8, linestyle=(0, (2, 2)))
    ax_lag.set_xlabel("frequency (Hz)", fontsize=9, color=MUTED)
    ax_lag.set_ylabel("lag (ms)", fontsize=9, color=MUTED)
    ax_lag.set_title("Closed-loop lag vs. frequency", fontsize=10.5, fontweight="bold")
    ax_lag.set_xticks(freqs)

    fig.suptitle("Closed-loop sine tracking, dac_y → cx, on-board setpoint generator "
                 f"(Kp=1.75, Ki=200, amplitude={amplitude_used:.2f}px / "
                 f"{amplitude_used*2*um:.1f}um pk-pk @ dac_y=2048)",
                 fontsize=13.5, fontweight="bold", y=1.02, color="#0b0b0b")

    out_png = args.out
    fig.savefig(out_png, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", out_png)


if __name__ == "__main__":
    main()
