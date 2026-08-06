#!/usr/bin/env python3
"""
Static DAC-counts -> amplifier-output-voltage characterization -- separate
from (and complementary to) the dynamic (frequency-dependent) sine-tracking
characterization in fta_sine_response_test_vcp.py's results.

Data is NOT collected automatically by this script -- unlike the other
fta_*.py scripts, which drive hardware and log a response, this records a
MANUALLY-measured dataset (DAC count commanded via set_x, amplifier output
voltage read by hand off a multimeter at each settled point) and just
fits/plots it. Recorded 2026-08-06: DAC 200-4000 in steps of 200 (20
points), amp output in volts. One reading (nominally the 11th point,
DAC=2200) came back as an outlier (0.85V) that breaks an otherwise very
consistent ~-0.147V per 200-count step -- treated as a bad/missing
reading and excluded rather than guessed at; see the commit/session log
for the reasoning. The remaining 20 raw values pair 1:1 with the 20 DAC
points once the bad one is dropped (removing it wasn't a replacement for
a real reading, it un-shifts the rest of the list back into correct
alignment).

Fits a straight line (V = slope*dac + intercept) via least-squares and
reports R^2 -- this is a genuinely linear, clean static transfer function
(R^2=0.997, no visible kink/dead-band across the full 200-4000 range).
Worth keeping in mind against the DYNAMIC nonlinear threshold effect found
in the sine-tracking amplitude comparison (severe rolloff at 15-20Hz with
small commanded amplitude, largely resolved at larger amplitude): this
static DC sweep suggests the DAC/amplifier stage itself isn't the source
of that nonlinearity, since it's this linear when swept slowly/statically
-- points toward something dynamic (mechanical stiction/backlash in the
actuator, or a frequency-dependent electrical effect) instead.

Also computes power dissipated in the load, P = V^2 / R, for the
documented load resistance (2.85 ohm) -- always non-negative, so this
comes out as a parabola-ish curve (roughly, since V vs DAC is linear)
centered near the zero-crossing DAC value, not a straight line.

Usage: python3 fta_amp_voltage_calibration.py
Outputs: results/fta_amp_voltage_calibration.png,
         results/fta_amp_voltage_calibration.npz
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOAD_OHMS = 2.85

# DAC counts, 200..4000 step 200 (20 points).
DAC_COUNTS = np.arange(200, 4000 + 1, 200)

# Amplifier output (V), manually read off a multimeter at each settled DAC
# point. The 11th raw reading (.85V) was a clear outlier breaking an
# otherwise consistent ~-0.147V/200-count trend -- excluded as bad/missing
# rather than guessed at (see module docstring).
AMP_VOLTS = np.array([
    1.43, 1.399, 1.263, 1.11, .968, .820, .672, .525, .378, .231,
    -.061, -.210, -.359, -.508, -.657, -.806, -.955, -1.104, -1.253, -1.388,
])

assert len(DAC_COUNTS) == len(AMP_VOLTS), (
    f"length mismatch: {len(DAC_COUNTS)} DAC points vs {len(AMP_VOLTS)} voltage readings")


def main():
    slope, intercept = np.polyfit(DAC_COUNTS, AMP_VOLTS, 1)
    fit_v = slope * DAC_COUNTS + intercept
    residuals = AMP_VOLTS - fit_v
    r2 = 1 - np.sum(residuals**2) / np.sum((AMP_VOLTS - AMP_VOLTS.mean())**2)
    zero_crossing_dac = -intercept / slope

    power_w = AMP_VOLTS**2 / LOAD_OHMS

    print(f"slope      = {slope:.6f} V/count  ({slope*1000:.4f} V/1000 counts)")
    print(f"intercept  = {intercept:.4f} V")
    print(f"R^2        = {r2:.5f}")
    print(f"zero-crossing (V=0) at DAC = {zero_crossing_dac:.1f} counts")
    print(f"power range = {power_w.min()*1000:.2f}mW to {power_w.max()*1000:.2f}mW "
          f"(R={LOAD_OHMS} ohm)")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), dpi=150)

    ax1.scatter(DAC_COUNTS, AMP_VOLTS, color="#2a78d6", s=35, zorder=3, label="measured")
    ax1.plot(DAC_COUNTS, fit_v, color="#898781", linewidth=1.5, linestyle="--", zorder=2,
              label=f"fit: V = {slope:.6f}×dac + {intercept:.3f}  (R²={r2:.4f})")
    ax1.axhline(0, color="#c3c2b7", linewidth=0.8)
    ax1.set_xlabel("DAC counts")
    ax1.set_ylabel("amplifier output (V)")
    ax1.set_title("DAC counts vs. amplifier output voltage")
    ax1.legend(fontsize=9)
    ax1.grid(True, color="#e1e0d9", linewidth=0.6)

    ax2.scatter(DAC_COUNTS, power_w * 1000, color="#eb6834", s=35, zorder=3)
    ax2.set_xlabel("DAC counts")
    ax2.set_ylabel("power into load (mW)")
    ax2.set_title(f"Power dissipated in load (R={LOAD_OHMS}Ω), P=V²/R")
    ax2.grid(True, color="#e1e0d9", linewidth=0.6)

    fig.tight_layout()
    out_png = "results/fta_amp_voltage_calibration.png"
    fig.savefig(out_png)
    print(f"wrote {out_png}")

    out_npz = "results/fta_amp_voltage_calibration.npz"
    np.savez(out_npz, dac_counts=DAC_COUNTS, amp_volts=AMP_VOLTS, power_w=power_w,
              load_ohms=LOAD_OHMS, slope=slope, intercept=intercept, r2=r2)
    print(f"wrote {out_npz}")


if __name__ == "__main__":
    main()
