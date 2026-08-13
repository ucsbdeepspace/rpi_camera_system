#!/usr/bin/env python3
"""
Second panel: the "push it farther" pass (2026-08-13, same day as the
first tuning panel) -- the Kp stability-boundary search (5.5/6.0/6.5/8.0,
Ki=0 to isolate) plus the best combined-gain candidates found afterward
(Kp=3.5 with Ki=200/300). Same visual style as fta_closed_loop_step_response_plot.py.

Usage: python3 fta_closed_loop_step_response_plot2.py
Output: results/fta_closed_loop_step_response_stability_panel.png
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from fta_closed_loop_step_response_vcp import analyze_step

RUNS = [
    ("results/fta_closed_loop_step_response_vcp_kp5500_ki0.npz", "Kp=5.5  Ki=0\nstable, damped"),
    ("results/fta_closed_loop_step_response_vcp_kp6000_ki0.npz", "Kp=6.0  Ki=0\nmarginal (heavy ringing)"),
    ("results/fta_closed_loop_step_response_vcp_kp6500_ki0.npz", "Kp=6.5  Ki=0\nUNSTABLE (growing)"),
    ("results/fta_closed_loop_step_response_vcp_kp8000_ki0.npz", "Kp=8.0  Ki=0\nUNSTABLE (growing)"),
    ("results/fta_closed_loop_step_response_vcp_kp3500_ki200000.npz", "Kp=3.5  Ki=200\nclean"),
    ("results/fta_closed_loop_step_response_vcp_kp3500_ki300000.npz", "Kp=3.5  Ki=300\nfast, some ringing"),
]

BLUE = "#2a78d6"
ORANGE = "#eb6834"
RED = "#c0392b"
TARGET_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"

UNSTABLE_LABELS = {"kp6500_ki0", "kp8000_ki0"}


def main():
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.5), dpi=180)
    fig.patch.set_facecolor("white")

    for ax, (path, label) in zip(axes.flat, RUNS):
        d = np.load(path)
        t, x, t_step = d["t"], d["x"], float(d["t_step"])
        target_from, target_to = int(d["target_from"]), int(d["target_to"])
        is_unstable = any(key in path for key in UNSTABLE_LABELS)
        metrics = None if is_unstable else analyze_step(t, x, t_step, 2.0)

        ax.set_facecolor("white")
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            ax.spines[spine].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=8.5, length=3)

        ax.axhline(target_from, color=TARGET_COLOR, linewidth=1.0, linestyle=(0, (2, 2)), alpha=0.7)
        ax.plot([t_step, t[-1]], [target_to, target_to], color=TARGET_COLOR,
                linewidth=1.1, linestyle=(0, (2, 2)))

        if is_unstable:
            color = RED
            t_lo, t_hi = t[0], t[-1]  # show the whole window -- sustained oscillation is the point
        else:
            color = ORANGE if (metrics and metrics["overshoot_pct"] and metrics["overshoot_pct"] > 5.0) else BLUE
            t_lo = max(t[0], t_step - 0.3)
            t_hi = min(t[-1], t_step + 0.8)
        mask = (t >= t_lo) & (t <= t_hi)
        ax.plot(t[mask], x[mask], color=color, linewidth=1.2)
        ax.axvline(t_step, color=MUTED, linewidth=0.7, linestyle=(0, (1, 2)))
        ax.set_xlim(t_lo, t_hi)

        ax.set_title(label, fontsize=10.5, fontweight="bold", color="#0b0b0b", pad=6)

        if metrics is not None:
            parts = []
            if metrics["rise_time_s"] is not None:
                parts.append(f"rise {metrics['rise_time_s']*1000:.0f}ms")
            if metrics["overshoot_pct"] is not None:
                parts.append(f"overshoot {metrics['overshoot_pct']:.1f}%")
            if metrics["settling_time_s"] is not None:
                parts.append(f"settle {metrics['settling_time_s']*1000:.0f}ms")
            else:
                parts.append("settle: n/a")
            txt = "  ·  ".join(parts)
        else:
            txt = "sustained oscillation -- never settles"
        ax.text(0.97, 0.95, txt, transform=ax.transAxes, fontsize=8,
                color="#0b0b0b", va="top", ha="right",
                bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=3))

        ax.set_xlabel("time (s)", fontsize=8.5, color=MUTED)
        ax.set_ylabel("cx (px)", fontsize=8.5, color=MUTED)

    clean_line = plt.Line2D([0], [0], color=BLUE, linewidth=1.5, label="clean (≤ 5% overshoot)")
    ring_line = plt.Line2D([0], [0], color=ORANGE, linewidth=1.5, label="visible ringing (> 5% overshoot)")
    unstable_line = plt.Line2D([0], [0], color=RED, linewidth=1.5, label="unstable (sustained/growing oscillation)")
    fig.legend(handles=[clean_line, ring_line, unstable_line], loc="upper center",
               bbox_to_anchor=(0.5, 1.03), ncol=3, frameon=False, fontsize=10)
    fig.suptitle("Kp stability-boundary search + best combined-gain candidates, dac_y → cx, -25px step @ dac_y=2048",
                 fontsize=13.5, fontweight="bold", y=1.10, color="#0b0b0b")
    fig.tight_layout()

    out_png = "results/fta_closed_loop_step_response_stability_panel.png"
    fig.savefig(out_png, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", out_png)


if __name__ == "__main__":
    main()
