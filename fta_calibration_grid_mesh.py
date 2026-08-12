#!/usr/bin/env python3
"""
Draws the commanded DAC grid as a polygonal mesh in centroid (pixel) space,
to sanity-check the per-axis projections used in fta_travel_range_analysis.py
(slide 18): those plots show cx vs. dac_x and cy vs. dac_y treated as if each
DAC channel only moves the corresponding pixel axis. But the sine-tracking
rotated-axis work (slides 10, 19, 20) already found the actuator's real
motion axis sits at roughly -150 to -157 degrees, nowhere near 0/90 -- so
dac_x moving "pure cx" is known to be an approximation, not exact.

This script skips projection assumptions entirely: it takes the raw
calibration grid (results/fta_calibration_vcp_*.npz, a clean NxM grid of
commanded (dac_x, dac_y) points) and connects points that share a dac_y
("rows", dac_x varies) and points that share a dac_x ("columns", dac_y
varies) directly in (cx, cy) pixel space. A perfectly linear, axis-aligned
actuator would draw a rectangular grid. Shear or rotation shows up as
non-perpendicular rows/columns; compression (saturation) shows up as rows or
columns bunching up instead of staying evenly spaced -- exactly the kind of
distortion that would be invisible in the per-axis slide-18 plots, which
only ever look at one DAC channel against one pixel axis at a time.

Usage: python3 fta_calibration_grid_mesh.py
Outputs: results/fta_calibration_grid_mesh.png
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_CALIB_PATH = "results/fta_calibration_vcp_20260806T191230Z.npz"  # 13x13, 200-3800 step 300

ROW_COLOR = "#2a78d6"    # constant dac_y, dac_x varies
COL_COLOR = "#eb6834"    # constant dac_x, dac_y varies
MUTED = "#898781"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--in", dest="calib_path", default=DEFAULT_CALIB_PATH)
    parser.add_argument("--out", dest="out_path", default="results/fta_calibration_grid_mesh.png")
    args = parser.parse_args()

    d = np.load(args.calib_path)
    dac_x, dac_y, cx, cy = d["dac_x"], d["dac_y"], d["cx"], d["cy"]

    ux = np.unique(dac_x)
    uy = np.unique(dac_y)
    print(f"grid: {len(ux)} dac_x values x {len(uy)} dac_y values = {len(ux)*len(uy)} points "
          f"({len(dac_x)} points actually captured)")

    fig, ax = plt.subplots(figsize=(9, 8.5), dpi=150)
    ax.set_facecolor("white")

    # rows: constant dac_y, points sorted by dac_x
    for i, y in enumerate(uy):
        mask = dac_y == y
        order = np.argsort(dac_x[mask])
        xs, ys = cx[mask][order], cy[mask][order]
        ax.plot(xs, ys, color=ROW_COLOR, linewidth=1.3, alpha=0.75, zorder=2,
                 marker="o", markersize=3)

    # columns: constant dac_x, points sorted by dac_y
    for j, x in enumerate(ux):
        mask = dac_x == x
        order = np.argsort(dac_y[mask])
        xs, ys = cx[mask][order], cy[mask][order]
        ax.plot(xs, ys, color=COL_COLOR, linewidth=1.3, alpha=0.75, zorder=2,
                 marker="o", markersize=3)

    # label the four corners with their commanded DAC values, so the mesh's
    # orientation/scale is readable directly off the plot.
    for xv in (ux[0], ux[-1]):
        for yv in (uy[0], uy[-1]):
            mask = (dac_x == xv) & (dac_y == yv)
            if mask.sum() != 1:
                continue
            px, py = cx[mask][0], cy[mask][0]
            ax.annotate(f"dac=({int(xv)},{int(yv)})", (px, py), fontsize=8, color="#0b0b0b",
                        xytext=(6, 6), textcoords="offset points",
                        bbox=dict(facecolor="white", edgecolor=MUTED, linewidth=0.6, pad=1.5))

    row_line = plt.Line2D([0], [0], color=ROW_COLOR, linewidth=1.5, marker="o", markersize=4,
                           label="constant dac_y (dac_x sweeps)")
    col_line = plt.Line2D([0], [0], color=COL_COLOR, linewidth=1.5, marker="o", markersize=4,
                           label="constant dac_x (dac_y sweeps)")
    ax.legend(handles=[row_line, col_line], fontsize=9.5, loc="best")

    ax.set_xlabel("cx (pixel)")
    ax.set_ylabel("cy (pixel)")
    ax.set_title("Commanded DAC grid drawn in centroid (pixel) space\n"
                  "— rectangular & evenly spaced = clean; sheared/bunched = distorted",
                  fontsize=12.5, fontweight="bold")
    ax.grid(True, color="#e1e0d9", linewidth=0.6)
    ax.set_aspect("equal", adjustable="datalim")

    fig.tight_layout()
    fig.savefig(args.out_path)
    print(f"wrote {args.out_path}")

    # quick numeric distortion checks: row spacing (dac_x direction) and
    # column spacing (dac_y direction) should each be roughly constant if
    # the mapping is linear -- print the min/max column-to-column pixel gap
    # per row, and vice versa, as a compression indicator.
    print("\nRow extents (cx,cy) span per dac_y value, i.e. how far dac_x sweeps the point "
          "in pixel space at each dac_y:")
    for y in uy:
        mask = dac_y == y
        order = np.argsort(dac_x[mask])
        xs, ys = cx[mask][order], cy[mask][order]
        span = np.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        print(f"  dac_y={int(y):5d}: total pixel-space travel across dac_x sweep = {span:7.1f}px")

    print("\nColumn extents (cx,cy) span per dac_x value, i.e. how far dac_y sweeps the point "
          "in pixel space at each dac_x:")
    for x in ux:
        mask = dac_x == x
        order = np.argsort(dac_y[mask])
        xs, ys = cx[mask][order], cy[mask][order]
        span = np.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        print(f"  dac_x={int(x):5d}: total pixel-space travel across dac_y sweep = {span:7.1f}px")


if __name__ == "__main__":
    main()
