#!/usr/bin/env python3
"""One-off comparison panel: Kd=0 baseline vs. Kd=0.001/0.005/0.05, all at
Kp=1.75/Ki=200, same -25px step @ dac_y=2048. Not a committed project
script -- ad hoc for the 2026-08-18 D-term evaluation."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE = "#2a78d6"
TARGET_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"

runs = [
    ("Kd=0.00 (baseline)", "results/fta_closed_loop_step_response_vcp_20260818T172003Z.npz"),
    ("Kd=0.001, fc=20Hz", "results/fta_closed_loop_step_response_kd1.npz"),
    ("Kd=0.005, fc=3Hz", "results/fta_closed_loop_step_response_kd5_fc3.npz"),
    ("Kd=0.001, fc=3Hz", "results/fta_closed_loop_step_response_kd1_fc3.npz"),
    ("Kd=0.001, fc=1Hz", "results/fta_closed_loop_step_response_kd1_fc1.npz"),
]

fig, axes = plt.subplots(1, 5, figsize=(21, 4.2), dpi=170, sharey=False)
for ax, (label, path) in zip(axes, runs):
    d = np.load(path)
    t, x = d["t"], d["x"]
    t_step = float(d["t_step"])
    target_from, target_to = float(d["target_from"]), float(d["target_to"])
    ax.set_facecolor("white")
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8, length=3)
    ax.axhline(target_from, color=TARGET_COLOR, linewidth=0.9, linestyle=(0, (2, 2)), alpha=0.6)
    ax.plot([t_step, t[-1]], [target_to, target_to], color=TARGET_COLOR, linewidth=1.0, linestyle=(0, (2, 2)))
    ax.plot(t, x, color=BLUE, linewidth=1.0)
    ax.axvline(t_step, color=MUTED, linewidth=0.7, linestyle=(0, (1, 2)))
    ax.set_title(label, fontsize=11, fontweight="bold")
    ax.set_xlabel("time (s)", fontsize=8.5, color=MUTED)
axes[0].set_ylabel("cx (px)", fontsize=9, color=MUTED)

fig.suptitle("D-term effect on closed-loop step response, dac_y → cx (Kp=1.75, Ki=200, -25px step @ dac_y=2048)\n"
             "lower fc converges back toward the Kd=0 baseline but never beats it",
             fontsize=13, fontweight="bold")
fig.tight_layout()
fig.savefig("results/fta_closed_loop_dterm_comparison.png", facecolor="white")
print("wrote results/fta_closed_loop_dterm_comparison.png")
