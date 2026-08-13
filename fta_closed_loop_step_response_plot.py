#!/usr/bin/env python3
"""
Combines all of the 2026-08-13 closed-loop step-response tuning-pass runs
(fta_closed_loop_step_response_vcp.py) into one multi-panel comparison
figure -- same visual style as fta_sine_response_plot.py.

Usage: python3 fta_closed_loop_step_response_plot.py
Output: results/fta_closed_loop_step_response_tuning_panel.png
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from fta_closed_loop_step_response_vcp import analyze_step, MICRONS_PER_PIXEL

RUNS = [
    ("results/fta_closed_loop_step_response_vcp_20260813T214602Z.npz", "Kp=1.75  Ki=30\n(original baseline)"),
    ("results/fta_closed_loop_step_response_vcp_kp5000.npz", "Kp=5.0  Ki=30"),
    ("results/fta_closed_loop_step_response_vcp_kp1750_ki100000.npz", "Kp=1.75  Ki=100"),
    ("results/fta_closed_loop_step_response_vcp_kp5000_ki100000.npz", "Kp=5.0  Ki=100"),
    ("results/fta_closed_loop_step_response_vcp_kp1750_ki200000.npz", "Kp=1.75  Ki=200\n(chosen)"),
    ("results/fta_closed_loop_step_response_vcp_kp1750_ki400000.npz", "Kp=1.75  Ki=400"),
]

BLUE = "#2a78d6"
ORANGE = "#eb6834"
TARGET_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"


def main():
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.5), dpi=180, sharey=False)
    fig.patch.set_facecolor("white")

    for ax, (path, label) in zip(axes.flat, RUNS):
        d = np.load(path)
        t, x, t_step = d["t"], d["x"], float(d["t_step"])
        target_from, target_to = int(d["target_from"]), int(d["target_to"])
        metrics = analyze_step(t, x, t_step, 2.0)

        ax.set_facecolor("white")
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            ax.spines[spine].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=8.5, length=3)

        ax.axhline(target_from, color=TARGET_COLOR, linewidth=1.0, linestyle=(0, (2, 2)), alpha=0.7)
        ax.plot([t_step, t[-1]], [target_to, target_to], color=TARGET_COLOR,
                linewidth=1.1, linestyle=(0, (2, 2)))
        # zoom the x-axis to the interesting window: a bit before the step
        # through ~1s past it, so fast runs aren't squashed into a sliver
        # and slow ones aren't cut off
        t_lo = max(t[0], t_step - 0.3)
        t_hi = min(t[-1], t_step + 1.2)
        mask = (t >= t_lo) & (t <= t_hi)

        overshoot_ok = metrics["overshoot_pct"] is not None and metrics["overshoot_pct"] > 5.0
        color = ORANGE if overshoot_ok else BLUE
        ax.plot(t[mask], x[mask], color=color, linewidth=1.3)
        ax.axvline(t_step, color=MUTED, linewidth=0.7, linestyle=(0, (1, 2)))
        ax.set_xlim(t_lo, t_hi)

        ax.set_title(label, fontsize=10.5, fontweight="bold", color="#0b0b0b", pad=6)

        parts = []
        if metrics["rise_time_s"] is not None:
            parts.append(f"rise {metrics['rise_time_s']*1000:.0f}ms")
        if metrics["overshoot_pct"] is not None:
            parts.append(f"overshoot {metrics['overshoot_pct']:.1f}%")
        if metrics["settling_time_s"] is not None:
            parts.append(f"settle {metrics['settling_time_s']*1000:.0f}ms")
        else:
            parts.append("settle: n/a")
        ax.text(0.97, 0.95, "  ·  ".join(parts), transform=ax.transAxes, fontsize=8,
                color="#0b0b0b", va="top", ha="right",
                bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=3))

        ax.set_xlabel("time (s)", fontsize=8.5, color=MUTED)
        ax.set_ylabel("cx (px)", fontsize=8.5, color=MUTED)

    clean_line = plt.Line2D([0], [0], color=BLUE, linewidth=1.5, label="clean (≤ 5% overshoot)")
    ring_line = plt.Line2D([0], [0], color=ORANGE, linewidth=1.5, label="visible ringing (> 5% overshoot)")
    fig.legend(handles=[clean_line, ring_line], loc="upper center",
               bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False, fontsize=10)
    fig.suptitle("Closed-loop step response tuning pass, dac_y → cx, -25px step @ dac_y=2048",
                 fontsize=14, fontweight="bold", y=1.08, color="#0b0b0b")
    fig.tight_layout()

    out_png = "results/fta_closed_loop_step_response_tuning_panel.png"
    fig.savefig(out_png, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", out_png)


if __name__ == "__main__":
    main()
