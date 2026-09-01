#!/usr/bin/env python3
"""One-off: model-free peak/trough-spacing frequency estimate for the
2026-08-19 ring-down data, with a plot marking the actual peaks used.
Not a committed project script -- ad hoc, replaces the less trustworthy
curve_fit-based estimate in fta_ringdown_test.py for this analysis."""
import numpy as np
from scipy.signal import find_peaks
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE = "#2a78d6"
ORANGE = "#eb6834"
MUTED = "#898781"
GRID = "#e1e0d9"

d = np.load("results/fta_ringdown_465hz_v3_20260819.npz")
t, x = d["t"], d["x"]

min_idx = int(np.argmin(x))
start_t = t[min_idx] + 0.075  # skip the messier initial 2-3 cycles (still settling
                                # from the forced-drive transient), start in the clean tail
mask = (t > start_t) & (t < start_t + 0.5)
tt, xx = t[mask], x[mask]

peak_idx, _ = find_peaks(xx, prominence=0.5)
trough_idx, _ = find_peaks(-xx, prominence=0.5)
pk_t = tt[peak_idx]
tr_t = tt[trough_idx]

pk_spacing = np.diff(pk_t)
tr_spacing = np.diff(tr_t)
all_spacing = np.concatenate([pk_spacing, tr_spacing])
mean_period = all_spacing.mean()
freq = 1.0 / mean_period

print(f"{len(pk_t)} peaks, {len(tr_t)} troughs")
print(f"mean spacing (peaks+troughs combined, n={len(all_spacing)}): {mean_period*1000:.2f}ms")
print(f"-> frequency = {freq:.2f}Hz")

fig, ax = plt.subplots(figsize=(11, 5), dpi=150)
ax.set_facecolor("white")
for spine in ("top", "right"):
    ax.spines[spine].set_visible(False)
for spine in ("left", "bottom"):
    ax.spines[spine].set_color(GRID)
ax.tick_params(colors=MUTED, labelsize=9, length=3)

ax.plot(t, x, color=BLUE, linewidth=1.1, label="measured cx")
ax.plot(tt[peak_idx], xx[peak_idx], "o", color=ORANGE, markersize=6, zorder=5, label="peaks used")
ax.plot(tt[trough_idx], xx[trough_idx], "o", color="#2a2a2a", markersize=6, zorder=5, label="troughs used")
ax.axvspan(start_t, start_t + 0.5, color=GRID, alpha=0.4, zorder=0, label="analysis window")

ax.set_xlim(t[min_idx] - 0.05, start_t + 0.55)
ax.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
ax.set_ylabel("cx (px)", fontsize=9.5, color=MUTED)
ax.legend(frameon=False, fontsize=9, loc="upper right")

parts = [f"{len(pk_t)} peaks + {len(tr_t)} troughs, mean spacing {mean_period*1000:.1f}ms",
         f"frequency = 1 / mean spacing = {freq:.2f}Hz"]
ax.text(0.02, 0.03, "\n".join(parts), transform=ax.transAxes, fontsize=10,
        color="#0b0b0b", va="bottom", ha="left", fontweight="bold",
        bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.95, pad=6))

fig.suptitle("Ring-down peak/trough spacing (model-free frequency estimate)", fontsize=13, fontweight="bold")
fig.tight_layout()
fig.savefig("results/fta_ringdown_peak_spacing_analysis.png", facecolor="white")
print("wrote results/fta_ringdown_peak_spacing_analysis.png")
