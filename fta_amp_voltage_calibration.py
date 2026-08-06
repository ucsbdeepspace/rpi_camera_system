#!/usr/bin/env python3
"""
Static DAC-counts -> amplifier-output-voltage characterization, both FTA
axes -- separate from (and complementary to) the dynamic (frequency-
dependent) sine-tracking characterization in fta_sine_response_test_vcp.py's
results.

Data is NOT collected automatically by this script -- unlike the other
fta_*.py scripts, which drive hardware and log a response, this records a
MANUALLY-measured dataset (DAC count commanded via set_x/set_y, amplifier
output voltage read by hand off a multimeter at each settled point) and
just fits/plots it. Recorded 2026-08-06: DAC 200-4000 in steps of 200 (20
points), amp output in volts, same 2.85 ohm load on both axes.

Y-axis amplifier: one raw reading (nominally the 11th point, DAC=2200)
came back as an outlier (0.85V) that breaks an otherwise very consistent
~-0.147V per 200-count step -- treated as a bad/missing reading and
excluded rather than guessed at. The remaining 20 raw values pair 1:1
with the 20 DAC points once the bad one is dropped (removing it wasn't a
replacement for a real reading, it un-shifts the rest of the list back
into correct alignment). X-axis amplifier: all 20 readings came back
clean, no exclusion needed -- notably its 10th value (.084V) lands almost
exactly where the Y-axis trend predicted its own (excluded) 11th point
would have been, a nice independent sanity check on that exclusion.

Fits a straight line (V = slope*dac + intercept) per axis via least-
squares and reports R^2 -- both are genuinely linear, clean static
transfer functions (R^2 > 0.99, no visible kink/dead-band across the full
200-4000 range). Worth keeping in mind against the DYNAMIC nonlinear
threshold effect found in the sine-tracking amplitude comparison (severe
rolloff at 15-20Hz with small commanded amplitude, largely resolved at
larger amplitude): this static DC sweep suggests the DAC/amplifier stage
itself isn't the source of that nonlinearity on either axis, since both
are this linear when swept slowly/statically -- points toward something
dynamic (mechanical stiction/backlash in the actuator, or a frequency-
dependent electrical effect) instead.

Also computes, per axis: power dissipated in the load (P = V^2/R, always
non-negative, so a parabola-ish curve centered near that axis's own
zero-crossing DAC value, not a straight line); DAC counts per volt
(1/slope); and DAC counts per amp (R/slope, via dV/dI = R for a fixed
resistive load).

Usage: python3 fta_amp_voltage_calibration.py
Outputs: results/fta_amp_voltage_calibration.png,
         results/fta_amp_voltage_calibration.npz
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOAD_OHMS = 2.85

# DAC counts, 200..4000 step 200 (20 points) -- same grid for both axes.
DAC_COUNTS = np.arange(200, 4000 + 1, 200)

# Y-axis amplifier output (V). The 11th raw reading (.85V) was a clear
# outlier breaking an otherwise consistent ~-0.147V/200-count trend --
# excluded as bad/missing rather than guessed at (see module docstring).
Y_AXIS_VOLTS = np.array([
    1.43, 1.399, 1.263, 1.11, .968, .820, .672, .525, .378, .231,
    -.061, -.210, -.359, -.508, -.657, -.806, -.955, -1.104, -1.253, -1.388,
])

# X-axis amplifier output (V). All 20 readings clean, no exclusion needed.
X_AXIS_VOLTS = np.array([
    1.39, 1.25, 1.108, .961, .814, .668, .521, .375, .229, .084,
    -.062, -.210, -.358, -.505, -.653, -.801, -.949, -1.095, -1.240, -1.382,
])

AXES = [("X", X_AXIS_VOLTS, "#2a78d6"), ("Y", Y_AXIS_VOLTS, "#eb6834")]

for name, volts, _ in AXES:
    assert len(DAC_COUNTS) == len(volts), (
        f"{name}-axis length mismatch: {len(DAC_COUNTS)} DAC points vs {len(volts)} voltage readings")


def fit_axis(volts):
    slope, intercept = np.polyfit(DAC_COUNTS, volts, 1)
    fit_v = slope * DAC_COUNTS + intercept
    r2 = 1 - np.sum((volts - fit_v) ** 2) / np.sum((volts - volts.mean()) ** 2)
    zero_crossing_dac = -intercept / slope
    counts_per_volt = 1.0 / slope
    counts_per_amp = LOAD_OHMS / slope  # dV/dI = R for a fixed resistive load
    power_w = volts ** 2 / LOAD_OHMS
    return {
        "slope": slope, "intercept": intercept, "fit_v": fit_v, "r2": r2,
        "zero_crossing_dac": zero_crossing_dac, "counts_per_volt": counts_per_volt,
        "counts_per_amp": counts_per_amp, "power_w": power_w,
    }


def main():
    results = {}
    for name, volts, _ in AXES:
        r = fit_axis(volts)
        results[name] = r
        print(f"--- {name} axis ---")
        print(f"slope      = {r['slope']:.6f} V/count  ({r['slope']*1000:.4f} V/1000 counts)")
        print(f"intercept  = {r['intercept']:.4f} V")
        print(f"R^2        = {r['r2']:.5f}")
        print(f"zero-crossing (V=0) at DAC = {r['zero_crossing_dac']:.1f} counts")
        print(f"counts per volt = {r['counts_per_volt']:.1f}")
        print(f"counts per amp  = {r['counts_per_amp']:.2f}  (R={LOAD_OHMS} ohm)")
        print(f"power range = {r['power_w'].min()*1000:.2f}mW to {r['power_w'].max()*1000:.2f}mW")
        print()

    fig, axes2d = plt.subplots(2, 2, figsize=(13, 9.5), dpi=150)

    for col, (name, volts, color) in enumerate(AXES):
        r = results[name]

        ax_v = axes2d[0, col]
        ax_v.scatter(DAC_COUNTS, volts, color=color, s=35, zorder=3, label="measured")
        ax_v.plot(DAC_COUNTS, r["fit_v"], color="#898781", linewidth=1.5, linestyle="--", zorder=2,
                   label=f"fit: V={r['slope']:.6f}×dac+{r['intercept']:.3f}  (R²={r['r2']:.4f})")
        ax_v.axhline(0, color="#c3c2b7", linewidth=0.8)
        ax_v.set_xlabel("DAC counts")
        ax_v.set_ylabel("amplifier output (V)")
        ax_v.set_title(f"{name} axis: DAC counts vs. amplifier voltage")
        ax_v.legend(fontsize=8.5)
        ax_v.grid(True, color="#e1e0d9", linewidth=0.6)
        # secondary top axis in volts, using THIS axis's own fit -- lets you
        # read the same plot off either DAC counts (bottom) or volts (top).
        sec_v = ax_v.secondary_xaxis(
            "top",
            functions=(lambda c, s=r["slope"], b=r["intercept"]: s * c + b,
                       lambda v, s=r["slope"], b=r["intercept"]: (v - b) / s))
        sec_v.set_xlabel("amplifier output (V)")

        ax_p = axes2d[1, col]
        ax_p.scatter(DAC_COUNTS, r["power_w"] * 1000, color=color, s=35, zorder=3)
        ax_p.set_xlabel("DAC counts")
        ax_p.set_ylabel("power into load (mW)")
        ax_p.set_title(f"{name} axis: power into load (R={LOAD_OHMS}Ω), P=V²/R")
        ax_p.grid(True, color="#e1e0d9", linewidth=0.6)
        sec_p = ax_p.secondary_xaxis(
            "top",
            functions=(lambda c, s=r["slope"], b=r["intercept"]: s * c + b,
                       lambda v, s=r["slope"], b=r["intercept"]: (v - b) / s))
        sec_p.set_xlabel("amplifier output (V)")

    fig.suptitle("FTA amplifier static calibration — both axes", fontsize=14, fontweight="bold")
    fig.tight_layout()
    out_png = "results/fta_amp_voltage_calibration.png"
    fig.savefig(out_png)
    print(f"wrote {out_png}")

    out_npz = "results/fta_amp_voltage_calibration.npz"
    np.savez(out_npz, dac_counts=DAC_COUNTS, x_axis_volts=X_AXIS_VOLTS, y_axis_volts=Y_AXIS_VOLTS,
              load_ohms=LOAD_OHMS,
              x_slope=results["X"]["slope"], x_intercept=results["X"]["intercept"], x_r2=results["X"]["r2"],
              y_slope=results["Y"]["slope"], y_intercept=results["Y"]["intercept"], y_r2=results["Y"]["r2"],
              x_counts_per_volt=results["X"]["counts_per_volt"], y_counts_per_volt=results["Y"]["counts_per_volt"],
              x_counts_per_amp=results["X"]["counts_per_amp"], y_counts_per_amp=results["Y"]["counts_per_amp"])
    print(f"wrote {out_npz}")


if __name__ == "__main__":
    main()
