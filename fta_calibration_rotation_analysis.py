#!/usr/bin/env python3
"""
Converts a saved fta_calibration_vcp.py grid-sweep (results/*.npz: dac_x,
dac_y, cx, cy, M, M_inv, rms_residual_px) into physical um/DAC-count gains
for each amplifier axis, and plots the result.

Why this needs its own script rather than just reading M's diagonal: the
camera frame and the amplifier's x/y axis LABELS are not aligned -- this
project has repeatedly found real rotation/cross-coupling between "dac_x"/
"dac_y" and the camera's cx/cy (see CLAUDE.md, e.g. the ~90 degree flip
found 2026-07-23, the near-anti-diagonal mapping found 2026-08-12, and the
still-open "backwards gain" investigation from 2026-09-10/11). So M[0,0]
("dac_x's effect on cx") alone is NOT dac_x's true displacement gain --
some of dac_x's real motion shows up in cy instead. The physically correct
per-axis gain is the length of that axis's full displacement VECTOR in
(cx,cy) space, independent of how that vector happens to be split between
the two pixel axes:

    dac_x gain (px/count) = || (M[0,0], M[1,0]) ||          (column 0 of M)
    dac_y gain (px/count) = || (M[0,1], M[1,1]) ||          (column 1 of M)

Each vector's direction (atan2) is the real motion axis for that amplifier
channel, measured relative to the camera's +cx axis -- the actual rotation
between "the axis labels on the amplifier" and "the camera frame." Converts
px -> um via MICRONS_PER_PIXEL (OV9281 pixel pitch, confirmed live via
Picamera2's own UnitCellSize elsewhere in this project) since the optics
are 1:1 (no external magnification), same constant/reasoning already used
throughout this project's other analysis scripts (fta_calibration_grid_mesh.py,
fta_sine_response_test_vcp.py, ...).

Usage: python3 fta_calibration_rotation_analysis.py [--in PATH] [--out-prefix PREFIX]
Outputs: <prefix>_mesh.png, <prefix>_vectors.png
"""
import argparse

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MICRONS_PER_PIXEL = 3.0  # OV9281 pixel pitch; optics are 1:1 so this applies
                          # directly to fiber-tip displacement too, no extra
                          # magnification factor -- see CLAUDE.md.

ROW_COLOR = "#2a78d6"   # constant dac_y, dac_x varies
COL_COLOR = "#eb6834"   # constant dac_x, dac_y varies
DACX_COLOR = "#2a78d6"
DACY_COLOR = "#eb6834"
MUTED = "#898781"


def axis_vector(M, col):
    """M is the 2x2 DAC->centroid gain block (px/count). Returns
    (magnitude_px, magnitude_um, angle_deg_from_camera_x) for DAC column
    `col` (0=dac_x, 1=dac_y) -- the real physical displacement per count
    along that amplifier channel, independent of the rotation between the
    amplifier's axis labels and the camera frame."""
    vec = M[:, col]
    mag_px = float(np.hypot(vec[0], vec[1]))
    angle_deg = float(np.degrees(np.arctan2(vec[1], vec[0])))
    return mag_px, mag_px * MICRONS_PER_PIXEL, angle_deg


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--in", dest="calib_path", default="results/fta_calibration_basic_20261103_tight.npz")
    parser.add_argument("--out-prefix", default=None)
    args = parser.parse_args()

    out_prefix = args.out_prefix or args.calib_path.rsplit(".", 1)[0]

    d = np.load(args.calib_path)
    dac_x, dac_y, cx, cy = d["dac_x"], d["dac_y"], d["cx"], d["cy"]
    M = d["M"]
    rms = float(d["rms_residual_px"])
    gain = M[:2, :2]  # px per DAC count

    mag_x_px, mag_x_um, ang_x = axis_vector(gain, 0)
    mag_y_px, mag_y_um, ang_y = axis_vector(gain, 1)
    # angle between the two amplifier axes' real motion directions -- 90 deg
    # would mean the two channels are mechanically orthogonal even though
    # each is rotated relative to the camera frame; a shear shows up as a
    # departure from 90.
    included_angle = abs(((ang_y - ang_x) + 180) % 360 - 180)

    print(f"Loaded {args.calib_path}: {len(dac_x)} points, RMS residual {rms:.2f}px")
    print(f"2x2 gain block (px/DAC-count):\n{gain}")
    print()
    print("Rotation-aware per-axis calibration (optics 1:1, "
          f"{MICRONS_PER_PIXEL:.1f}um/px pixel pitch):")
    print(f"  dac_x: {mag_x_px:.4f} px/count = {mag_x_um:.3f} um/count, "
          f"real motion direction {ang_x:+7.2f} deg from camera +cx axis")
    print(f"  dac_y: {mag_y_px:.4f} px/count = {mag_y_um:.3f} um/count, "
          f"real motion direction {ang_y:+7.2f} deg from camera +cx axis")
    print(f"  angle between dac_x's and dac_y's real motion axes: "
          f"{included_angle:.2f} deg (90 = mechanically orthogonal, "
          "no shear)")
    print(f"  rotation of dac_y's axis away from the camera's +cy axis "
          f"(90 deg): {ang_y - 90:+.2f} deg")

    # ---- plot 1: grid mesh in pixel space (rows/columns of the commanded
    # DAC grid, same convention as fta_calibration_grid_mesh.py) ----
    ux, uy = np.unique(dac_x), np.unique(dac_y)
    fig, ax = plt.subplots(figsize=(8.5, 8), dpi=150)
    for y in uy:
        mask = dac_y == y
        order = np.argsort(dac_x[mask])
        ax.plot(cx[mask][order], cy[mask][order], color=ROW_COLOR, linewidth=1.4,
                alpha=0.8, marker="o", markersize=4, zorder=2)
    for x in ux:
        mask = dac_x == x
        order = np.argsort(dac_y[mask])
        ax.plot(cx[mask][order], cy[mask][order], color=COL_COLOR, linewidth=1.4,
                alpha=0.8, marker="o", markersize=4, zorder=2)
    for xv in (ux[0], ux[-1]):
        for yv in (uy[0], uy[-1]):
            m = (dac_x == xv) & (dac_y == yv)
            if m.sum() != 1:
                continue
            ax.annotate(f"dac=({int(xv)},{int(yv)})", (cx[m][0], cy[m][0]), fontsize=8,
                        xytext=(6, 6), textcoords="offset points",
                        bbox=dict(facecolor="white", edgecolor=MUTED, linewidth=0.6, pad=1.5))
    ax.legend(handles=[
        plt.Line2D([0], [0], color=ROW_COLOR, marker="o", markersize=4, label="constant dac_y (dac_x sweeps)"),
        plt.Line2D([0], [0], color=COL_COLOR, marker="o", markersize=4, label="constant dac_x (dac_y sweeps)"),
    ], fontsize=9, loc="best")
    ax.set_xlabel("cx (pixel)")
    ax.set_ylabel("cy (pixel)")
    ax.set_title(f"Basic calibration grid, 0.5s settle/point -- {len(dac_x)} pts, RMS={rms:.1f}px\n"
                 f"{MICRONS_PER_PIXEL:.1f}um/px, optics 1:1", fontsize=12, fontweight="bold")
    ax.grid(True, color="#e1e0d9", linewidth=0.6)
    ax.set_aspect("equal", adjustable="datalim")
    sx = ax.secondary_xaxis("top", functions=(lambda p: p * MICRONS_PER_PIXEL, lambda u: u / MICRONS_PER_PIXEL))
    sx.set_xlabel("cx (um)")
    sy = ax.secondary_yaxis("right", functions=(lambda p: p * MICRONS_PER_PIXEL, lambda u: u / MICRONS_PER_PIXEL))
    sy.set_ylabel("cy (um)")
    fig.tight_layout()
    mesh_path = f"{out_prefix}_mesh.png"
    fig.savefig(mesh_path)
    print(f"\nwrote {mesh_path}")

    # ---- plot 2: vector diagram -- shows the real motion direction of
    # each amplifier axis (one DAC count's worth of travel, scaled up for
    # visibility) against the camera's own x/y axes, making the rotation
    # between "the axis labels on the amplifier" and the camera frame
    # directly visible. ----
    fig2, ax2 = plt.subplots(figsize=(7.2, 7.2), dpi=150)
    scale = 200.0  # um per count -> um shown, i.e. a 200-DAC-count step
    vx = gain[:, 0] * MICRONS_PER_PIXEL * scale
    vy = gain[:, 1] * MICRONS_PER_PIXEL * scale
    ax2.annotate("", xy=(vx[0], vx[1]), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=DACX_COLOR, linewidth=2.5))
    ax2.annotate("", xy=(vy[0], vy[1]), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=DACY_COLOR, linewidth=2.5))
    ax2.text(vx[0] * 1.08, vx[1] * 1.08,
              f"dac_x\n{mag_x_um:.2f} um/count\n{ang_x:+.1f} deg",
              color=DACX_COLOR, fontsize=9.5, fontweight="bold", ha="center", va="center")
    ax2.text(vy[0] * 1.08, vy[1] * 1.08,
              f"dac_y\n{mag_y_um:.2f} um/count\n{ang_y:+.1f} deg",
              color=DACY_COLOR, fontsize=9.5, fontweight="bold", ha="center", va="center")
    # camera frame reference axes
    lim = 1.4 * max(np.hypot(vx[0], vx[1]), np.hypot(vy[0], vy[1]))
    ax2.annotate("", xy=(lim * 0.55, 0), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=MUTED, linewidth=1.2, linestyle="--"))
    ax2.annotate("", xy=(0, lim * 0.55), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=MUTED, linewidth=1.2, linestyle="--"))
    ax2.text(lim * 0.58, 0, "camera +cx", color=MUTED, fontsize=9, va="center")
    ax2.text(0, lim * 0.58, "camera +cy", color=MUTED, fontsize=9, ha="center")
    ax2.set_xlim(-lim, lim)
    ax2.set_ylim(-lim, lim)
    ax2.set_aspect("equal")
    ax2.axhline(0, color="#d8d6cf", linewidth=0.6, zorder=0)
    ax2.axvline(0, color="#d8d6cf", linewidth=0.6, zorder=0)
    ax2.set_xlabel("um (camera x)")
    ax2.set_ylabel("um (camera y)")
    ax2.set_title("Real motion direction of each amplifier DAC axis\n"
                   f"(arrows scaled to a {scale:.0f}-count step; angle between axes =\n"
                   f"{included_angle:.1f} deg)", fontsize=11, fontweight="bold")
    fig2.tight_layout()
    vec_path = f"{out_prefix}_vectors.png"
    fig2.savefig(vec_path)
    print(f"wrote {vec_path}")


if __name__ == "__main__":
    main()
