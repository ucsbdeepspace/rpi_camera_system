#!/usr/bin/env python3
"""
Combines two already-collected datasets to find the actuator's real
mechanical travel limits and how much power gets spent getting there:

  - fta_calibration_vcp.py's DAC->centroid sweep (results/fta_calibration_vcp_*.npz)
    -- where does the beam centroid actually move vs. DAC count?
  - fta_amp_voltage_calibration.py's DAC->amplifier-voltage fit
    (results/fta_amp_voltage_calibration.npz) -- converted to power via
    P = V^2/R for the same load (2.85 ohm).

The mechanical question this answers: at some DAC value near each end of
the range, the centroid should stop moving (a real mechanical hard stop
or the beam leaving the flexure's usable travel) even though the DAC/amp
voltage keeps changing linearly right up to the clamp (already confirmed
very linear statically, see fta_amp_voltage_calibration.py's docstring).
Plotting centroid position and power on the same DAC-count axis shows
directly whether the calibration sweep's tested range (200-3800) actually
reached that flattening point, or whether the beam was still moving
right at the edges -- and, either way, how much power the amp was
already spending out there.

Uses the WIDER of the two calibration sweeps (200-3800 DAC, both axes,
169 points) for maximum range coverage. cx is plotted against dac_x and
cy against dac_y -- matching the already-established finding that each
axis's own DAC channel is by far its dominant driver (see the
axis-coupling-flip section in CLAUDE.md); the cross-coupled component is
real but secondary and not the point of this particular plot.

Usage: python3 fta_travel_range_analysis.py
Outputs: results/fta_travel_range_analysis.png
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CALIB_PATH = "results/fta_calibration_vcp_20260806T191230Z.npz"  # 200-3800, widest range
AMP_PATH = "results/fta_amp_voltage_calibration.npz"


def main():
    calib = np.load(CALIB_PATH)
    dac_x, dac_y, cx, cy = calib["dac_x"], calib["dac_y"], calib["cx"], calib["cy"]

    amp = np.load(AMP_PATH)
    load_ohms = float(amp["load_ohms"])
    fits = {
        "x": (float(amp["x_slope"]), float(amp["x_intercept"])),
        "y": (float(amp["y_slope"]), float(amp["y_intercept"])),
    }

    dac_grid = np.arange(200, 3800 + 1, 100)

    def power_curve(axis):
        slope, intercept = fits[axis]
        v = slope * dac_grid + intercept
        return v ** 2 / load_ohms

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), dpi=150)

    for col, (axis_name, dac_vals, centroid_vals, color) in enumerate([
        ("x", dac_x, cx, "#2a78d6"),
        ("y", dac_y, cy, "#eb6834"),
    ]):
        # per-DAC-value average centroid position, collapsing the OTHER
        # axis's grid variation -- reveals the trend more clearly than the
        # raw scatter alone, without hiding the real spread.
        uniq = np.unique(dac_vals)
        avg = np.array([centroid_vals[dac_vals == u].mean() for u in uniq])

        ax_c = axes[0, col]
        ax_c.scatter(dac_vals, centroid_vals, color=color, s=12, alpha=0.35, zorder=2,
                      label="all grid points")
        ax_c.plot(uniq, avg, color=color, linewidth=2.2, marker="o", markersize=5, zorder=3,
                   label="mean at each DAC value")
        ax_c.set_xlabel(f"DAC-{axis_name} counts")
        ax_c.set_ylabel(f"c{axis_name} (pixel position)")
        ax_c.set_title(f"{axis_name.upper()} axis: centroid vs. DAC-{axis_name}")
        ax_c.legend(fontsize=8.5)
        ax_c.grid(True, color="#e1e0d9", linewidth=0.6)
        slope, intercept = fits[axis_name]
        sec = ax_c.secondary_xaxis(
            "top",
            functions=(lambda c, s=slope, b=intercept: s * c + b,
                       lambda v, s=slope, b=intercept: (v - b) / s))
        sec.set_xlabel("amplifier output (V)")

        ax_p = axes[1, col]
        power_mw = power_curve(axis_name) * 1000
        ax_p.plot(dac_grid, power_mw, color=color, linewidth=2)
        ax_p.set_xlabel(f"DAC-{axis_name} counts")
        ax_p.set_ylabel("power into load (mW)")
        ax_p.set_title(f"{axis_name.upper()} axis: power vs. DAC-{axis_name} (R={load_ohms}Ω)")
        ax_p.grid(True, color="#e1e0d9", linewidth=0.6)
        sec2 = ax_p.secondary_xaxis(
            "top",
            functions=(lambda c, s=slope, b=intercept: s * c + b,
                       lambda v, s=slope, b=intercept: (v - b) / s))
        sec2.set_xlabel("amplifier output (V)")

        # print a quick numeric check for flattening at each end: compare
        # the centroid delta over the first/last 20% of the tested range
        # against the middle 20%, as a rough saturation indicator.
        span = uniq.max() - uniq.min()
        lo_mask = uniq <= uniq.min() + 0.2 * span
        hi_mask = uniq >= uniq.max() - 0.2 * span
        mid_mask = (uniq >= uniq.min() + 0.4 * span) & (uniq <= uniq.min() + 0.6 * span)
        def px_per_count(mask):
            if mask.sum() < 2:
                return float("nan")
            return (avg[mask][-1] - avg[mask][0]) / (uniq[mask][-1] - uniq[mask][0])
        print(f"{axis_name.upper()} axis: px/count  low-end={px_per_count(lo_mask):+.4f}  "
              f"mid={px_per_count(mid_mask):+.4f}  high-end={px_per_count(hi_mask):+.4f}  "
              f"(closer to 0 at an end = flattening/saturating there)")

    fig.suptitle("Where does the centroid actually stop moving, and what's the power cost getting there?",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    out_png = "results/fta_travel_range_analysis.png"
    fig.savefig(out_png)
    print(f"\nwrote {out_png}")


if __name__ == "__main__":
    main()
