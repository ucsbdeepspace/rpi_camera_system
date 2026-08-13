#!/usr/bin/env python3
"""
Plots the dac_y->cx sine-tracking results (results/fta_sine_response_vcp_y_roi_*.npz)
-- the ROI-telemetry-rate re-check that superseded the earlier full-frame
run (see CLAUDE.md, 2026-08-12). Two figures:

  1. Per-frequency traces: commanded dac_y on top, measured cx (the pixel
     axis this pathway actually drives, per the locked-optics calibration)
     on the bottom, one column per frequency -- same visual style as the
     slide-10 rotated-axis plots used throughout this project.
  2. Summary: true displacement magnitude and direction vs. frequency
     across the sweep, to show the flat-then-rises-at-20Hz magnitude shape
     and the direction staying pinned near 0 degrees (pure camera-x).

Usage: python3 fta_sine_response_plot.py
Outputs: results/fta_sine_response_y_roi_traces.png,
         results/fta_sine_response_y_roi_summary.png
"""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FREQS = [5, 10, 15, 20]
FILES = [f"results/fta_sine_response_vcp_y_roi_{f}Hz.npz" for f in FREQS]

BLUE = "#2a78d6"
DAC_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"


def plot_traces(runs):
    fig, axes = plt.subplots(
        2, 4, figsize=(13.33, 5.6), dpi=200,
        gridspec_kw={"height_ratios": [1, 2.4], "hspace": 0.12, "wspace": 0.30},
    )
    fig.patch.set_facecolor("white")

    for col, run in enumerate(runs):
        ax_dac = axes[0, col]
        ax_px = axes[1, col]
        for ax in (ax_dac, ax_px):
            ax.set_facecolor("white")
            for spine in ("top", "right"):
                ax.spines[spine].set_visible(False)
            for spine in ("left", "bottom"):
                ax.spines[spine].set_color(GRID)
            ax.tick_params(colors=MUTED, labelsize=8, length=3)

        ax_dac.plot(run["cmd_t"], run["cmd_v"], color=DAC_COLOR, linewidth=1.2)
        ax_dac.set_title(f"{run['freq']} Hz", fontsize=11, fontweight="bold",
                          color="#0b0b0b", pad=8)
        if col == 0:
            ax_dac.set_ylabel("commanded\ndac_y (counts)", fontsize=8.5, color=MUTED)
        ax_dac.set_xticklabels([])

        ax_px.plot(run["t"], run["x"], color=BLUE, linewidth=1.3)
        ax_px.set_xlabel("time (s)", fontsize=8.5, color=MUTED)
        if col == 0:
            ax_px.set_ylabel("cx (px)\n(driven by dac_y)", fontsize=8.5, color=MUTED)

        subtitle = (f"|gain|: {run['vector_mag']:.1f}px  ·  "
                    f"dir: {run['vector_angle_deg']:.1f}°")
        ax_px.text(0.02, 0.03, subtitle, transform=ax_px.transAxes, fontsize=7.8,
                   color=MUTED, va="bottom", ha="left",
                   bbox=dict(facecolor="white", edgecolor="none", alpha=0.8, pad=1.5))

    dac_line = plt.Line2D([0], [0], color=DAC_COLOR, linewidth=1.2, label="commanded dac_y")
    cx_line = plt.Line2D([0], [0], color=BLUE, linewidth=1.3, label="measured cx")
    fig.legend(handles=[dac_line, cx_line], loc="upper center",
               bbox_to_anchor=(0.5, 1.05), ncol=2, frameon=False, fontsize=9.5)
    fig.suptitle("dac_y → cx sine tracking, amplitude 400, ROI telemetry rate (~207Hz)",
                 fontsize=12.5, fontweight="bold", y=1.16, color="#0b0b0b")

    out_png = "results/fta_sine_response_y_roi_traces.png"
    fig.savefig(out_png, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", out_png)


def plot_summary(runs):
    freqs = [r["freq"] for r in runs]
    mags = [r["vector_mag"] for r in runs]
    angles = [r["vector_angle_deg"] for r in runs]

    fig, (ax_mag, ax_ang) = plt.subplots(1, 2, figsize=(10, 4), dpi=150)
    for ax in (ax_mag, ax_ang):
        ax.set_facecolor("white")
        ax.grid(True, color=GRID, linewidth=0.6)

    ax_mag.plot(freqs, mags, color=BLUE, linewidth=2, marker="o", markersize=6)
    ax_mag.set_xlabel("frequency (Hz)")
    ax_mag.set_ylabel("true displacement magnitude (px)")
    ax_mag.set_title("Gain vs. frequency")
    ax_mag.set_xticks(freqs)

    ax_ang.plot(freqs, angles, color="#eb6834", linewidth=2, marker="o", markersize=6)
    ax_ang.axhline(0, color=MUTED, linewidth=0.8, linestyle=(0, (2, 2)))
    ax_ang.set_xlabel("frequency (Hz)")
    ax_ang.set_ylabel("direction (deg, 0=pure camera-x)")
    ax_ang.set_title("Direction vs. frequency")
    ax_ang.set_xticks(freqs)

    fig.suptitle("dac_y → cx sine tracking summary (amplitude 400, ROI telemetry rate)",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    out_png = "results/fta_sine_response_y_roi_summary.png"
    fig.savefig(out_png)
    print("wrote", out_png)


def main():
    runs = []
    for path in FILES:
        d = np.load(path)
        runs.append({
            "freq": int(d["freq"]),
            "t": d["t"], "x": d["x"], "cmd_t": d["cmd_t"], "cmd_v": d["cmd_v"],
            "vector_mag": float(d["vector_mag"]),
            "vector_angle_deg": float(d["vector_angle_deg"]),
        })
    plot_traces(runs)
    plot_summary(runs)


if __name__ == "__main__":
    main()
