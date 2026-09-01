import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE = "#2a78d6"
ORANGE = "#eb6834"
TARGET_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"

freqs = [5, 10, 15, 20]
nonotch_files = [f"results/scratch_sine_nonotch_ki500_{f}Hz.npz" for f in freqs]
notch_files = [f"results/scratch_sine_notch_ki400_{f}Hz.npz" for f in freqs]

nonotch_gain = [float(np.load(p)["gain"]) for p in nonotch_files]
notch_gain = [float(np.load(p)["gain"]) for p in notch_files]

fig = plt.figure(figsize=(9.6, 5.9), dpi=150)
gs = fig.add_gridspec(1, 2, width_ratios=[1.05, 1], wspace=0.32)

# --- left: gain (T) vs frequency, both configs ---
ax1 = fig.add_subplot(gs[0])
ax1.set_facecolor("white")
for spine in ("top", "right"):
    ax1.spines[spine].set_visible(False)
for spine in ("left", "bottom"):
    ax1.spines[spine].set_color(GRID)
ax1.tick_params(colors=MUTED, labelsize=9, length=3)
ax1.grid(axis="y", color=GRID, linewidth=0.8, zorder=0)

ax1.plot(freqs, nonotch_gain, color=BLUE, marker="o", linewidth=2, markersize=7,
          label="no notch, Ki=500\n(each config's own max-speed Ki)", zorder=3)
ax1.plot(freqs, notch_gain, color=ORANGE, marker="s", linewidth=2, markersize=7,
          label="notch@38.5Hz, Ki=400", zorder=3)
for f, g in zip(freqs, nonotch_gain):
    ax1.annotate(f"{g:.2f}", (f, g), textcoords="offset points", xytext=(0, 9),
                 fontsize=8.5, color=BLUE, ha="center")
for f, g in zip(freqs, notch_gain):
    ax1.annotate(f"{g:.2f}", (f, g), textcoords="offset points", xytext=(0, -14),
                 fontsize=8.5, color=ORANGE, ha="center")

ax1.axhline(1.0, color=MUTED, linewidth=0.8, linestyle=(0, (2, 2)))
ax1.set_ylim(0, 1.08)
ax1.set_xticks(freqs)
ax1.set_xlabel("frequency (Hz)", fontsize=9.5, color=MUTED)
ax1.set_ylabel("tracking gain T", fontsize=9.5, color=MUTED)
ax1.set_title("T vs frequency — each config at its own\nmax stable Ki", fontsize=11, color="#1A1A2E",
              loc="left", pad=10)
ax1.legend(frameon=False, fontsize=8.5, loc="lower left")

# --- right: 15Hz raw trace overlay, where the two configs diverge most ---
ax2 = fig.add_subplot(gs[1])
ax2.set_facecolor("white")
for spine in ("top", "right"):
    ax2.spines[spine].set_visible(False)
for spine in ("left", "bottom"):
    ax2.spines[spine].set_color(GRID)
ax2.tick_params(colors=MUTED, labelsize=9, length=3)

d_non = np.load("results/scratch_sine_nonotch_ki500_15Hz.npz")
d_notch = np.load("results/scratch_sine_notch_ki400_15Hz.npz")

t_non = d_non["t"] - d_non["t"][0]
t_notch = d_notch["t"] - d_notch["t"][0]
window = 0.6  # seconds, enough to show a few cycles at 15Hz
mask_non = t_non < window
mask_notch = t_notch < window

ax2.plot(t_non[mask_non], d_non["tgt"][mask_non], color=TARGET_COLOR, linewidth=1.1,
          linestyle=(0, (2, 2)), label="commanded target_x", zorder=2)
ax2.plot(t_non[mask_non], d_non["x"][mask_non], color=BLUE, linewidth=1.6,
          label=f"no notch, Ki=500 (T={float(d_non['gain']):.2f})", zorder=3)
ax2.plot(t_notch[mask_notch], d_notch["x"][mask_notch], color=ORANGE, linewidth=1.6,
          label=f"notch, Ki=400 (T={float(d_notch['gain']):.2f})", zorder=3)

ax2.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
ax2.set_ylabel("cx (px)", fontsize=9.5, color=MUTED)
ax2.set_title("Raw 15Hz trace — where notch loses\nmost of its signal", fontsize=11, color="#1A1A2E",
              loc="left", pad=10)
ax2.legend(frameon=False, fontsize=8, loc="upper right")

fig.savefig("results/scratch_notch_maxspeed_comparison.png", facecolor="white", bbox_inches="tight")
print("saved results/scratch_notch_maxspeed_comparison.png")
