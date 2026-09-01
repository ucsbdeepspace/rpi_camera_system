#!/usr/bin/env python3
"""
Ad hoc (not committed-quality) combined summary for the 2026-08-19 notch-
retest + Ki=400 pass: per-frequency traces (old Ki=200/no-notch vs new
Ki=400/notch@38.5Hz, both throttled 200Hz) plus a gain/sensitivity vs.
frequency comparison -- the single image summarizing this pass.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE = "#2a78d6"
GRAY = "#b7b3ab"
TARGET_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
UM = 3.0

FREQS = [5, 10, 15, 20]
OLD_FMT = "results/fta_closed_loop_onboard_sine_throttle200_ki200_{f}Hz_15umpp.npz"
NEW_FMT = "results/fta_sine_notch_ki400_{f}Hz_15umpp.npz"


def load(path):
    d = np.load(path)
    return {"t": d["t"], "x": d["x"], "tgt": d["tgt"], "gain": float(d["gain"])}


def style(ax):
    ax.set_facecolor("white")
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8, length=3)


def main():
    olds = {f: load(OLD_FMT.format(f=f)) for f in FREQS}
    news = {f: load(NEW_FMT.format(f=f)) for f in FREQS}

    fig = plt.figure(figsize=(16, 7.5), dpi=180)
    fig.patch.set_facecolor("white")
    gs = fig.add_gridspec(2, len(FREQS), height_ratios=[1.3, 1], hspace=0.5, wspace=0.3)

    for col, f in enumerate(FREQS):
        ax = fig.add_subplot(gs[0, col])
        style(ax)
        o, n = olds[f], news[f]
        # trim old trace to roughly the same window length as new for a fair visual
        ax.plot(o["t"], o["x"] * UM, color=GRAY, linewidth=1.1, label="old: Ki=200, no notch")
        ax.plot(n["t"], n["x"] * UM, color=BLUE, linewidth=1.3, label="new: Ki=400 + notch")
        ax.plot(n["t"], n["tgt"] * UM, color=TARGET_COLOR, linewidth=0.9,
                linestyle=(0, (2, 2)), label="target")
        ax.set_title(f"{f}Hz", fontsize=12, fontweight="bold", color="#0b0b0b")
        ax.set_xlabel("time (s)", fontsize=8, color=MUTED)
        if col == 0:
            ax.set_ylabel("cx (µm)", fontsize=8.5, color=MUTED)
        ax.text(0.03, 0.03, f"old T={o['gain']:.2f}\nnew T={n['gain']:.2f}",
                transform=ax.transAxes, fontsize=8, color="#0b0b0b", va="bottom", ha="left",
                bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=3))

    fig.legend(
        handles=[
            plt.Line2D([0], [0], color=GRAY, linewidth=1.6, label="old: Ki=200, no notch"),
            plt.Line2D([0], [0], color=BLUE, linewidth=1.8, label="new: Ki=400 + notch"),
            plt.Line2D([0], [0], color=TARGET_COLOR, linewidth=1.2, linestyle=(0, (2, 2)), label="target"),
        ],
        loc="upper center", bbox_to_anchor=(0.5, 0.965), ncol=3, frameon=False, fontsize=10,
    )

    ax_gain = fig.add_subplot(gs[1, 1:3])
    style(ax_gain)
    ax_gain.grid(True, color=GRID, linewidth=0.6)

    old_gains = [olds[f]["gain"] for f in FREQS]
    new_gains = [news[f]["gain"] for f in FREQS]

    ax_gain.plot(FREQS, old_gains, color=GRAY, linewidth=2, marker="o", markersize=6,
                 label="old: Ki=200, no notch")
    ax_gain.plot(FREQS, new_gains, color=BLUE, linewidth=2, marker="o", markersize=6,
                 label="new: Ki=400 + notch@38.5Hz")
    ax_gain.axhline(1.0, color="#8a1f1f", linewidth=1.0, linestyle=(0, (2, 2)), label="unity (T=1)")
    ax_gain.set_xlabel("frequency (Hz)", fontsize=9.5, color=MUTED)
    ax_gain.set_ylabel("tracking gain (T)", fontsize=9.5, color=MUTED)
    ax_gain.set_title("Tracking gain vs. frequency", fontsize=11, fontweight="bold")
    ax_gain.set_xticks(FREQS)
    ax_gain.set_ylim(0, 1.05)
    ax_gain.legend(frameon=False, fontsize=8.5, loc="upper right")

    fig.suptitle("Notch + Ki=400 pass: real improvement across 5-20Hz, not yet unity gain\n"
                 "(dac_y=2048, throttled 200Hz, 2.5px/15um pk-pk)",
                 fontsize=14, fontweight="bold", y=1.08, color="#0b0b0b")

    out = "results/fta_notch_ki400_pass_combined.png"
    fig.savefig(out, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main()
