#!/usr/bin/env python3
"""
Sweeps the FTA over a grid of DAC setpoints and records the camera-observed
beam centroid at each point, then fits a 3x3 affine matrix (homogeneous 2x2
gain + offset) mapping DAC setpoint -> centroid pixel, plus its inverse
(centroid -> DAC setpoint) for later closed-loop use.

Drives the sweep via the Nucleo's EXISTING `grid_scan x1 y1 x2 y2` serial
command (see "FTA Controller" firmware, ucsbdeepspace/7-element-array branch
lock_in_2 -- grid_scan_roi() in main.c) -- no firmware changes needed.  That
routine already does smooth, backlash-aware microstep travel and per-point
settling, and streams one `SD <adc> <x_center> <y_center>\n` line per sampled
point (sampling only on the downward Y pass, to avoid up/down hysteresis
asymmetry). This script treats each SD line purely as a capture TRIGGER: by
the time it arrives, the position has already been sitting still for the
firmware's own dwell time (currently a hardcoded ~50ms + travel time), so the
frame(s) captured right after receiving it should reflect a settled position.
The `<adc>` value itself is NOT used for the fit -- it's that firmware's own
onboard lock-in/photodiode reading, a different physical sensor than the
camera -- but it's logged alongside in case cross-referencing it is ever
useful.

Talks over the Nucleo's own USB-serial link (460800 baud, matches
FTA_GUI_PID.py's existing host driver and huart2's config in "FTA
Controller"/Core/Src/main.c in that same repo) -- NOT the I2C link used
elsewhere in this project (nucleo_i2c_sender.py) for streaming centroids from
camera_view_tool.py. This is a different physical connection (Nucleo USB
straight into this Pi) and, as of 2026-07-23, possibly a different physical
Nucleo board / firmware image than the one wired for I2C centroid receiving --
those two roles haven't been consolidated yet.

Beam detection (find_beam_blob) is intentionally duplicated from
camera_view_tool.py / beam_position_streamer.py rather than imported -- see
beam_position_streamer.py's module docstring for why (no __main__ guard on
camera_view_tool.py). Mirror any confidence-gate constant changes here too.

Usage:
  python3 fta_calibration.py X1 Y1 X2 Y2 [--grid-step N] [--raw-size WxH]
      [--y-start N] [--port PORT] [--frames-per-point N] [--out PATH]
      [--dry-run]

    X1 Y1 X2 Y2         DAC-count grid corners (0-4095, firmware clamps and
                         reorders them itself) -- required, no default, so a
                         safe sweep range is a deliberate per-setup choice,
                         not an assumed one. Firmware's own default safety
                         ROI is 95-4000 -- start narrower than that on a
                         rig/mount that hasn't been swept before.
    --grid-step N        DAC counts between grid samples (sent as
                         set_grid_step_size before the scan). Default: don't
                         send it, firmware keeps its own default (100).
    --raw-size WxH       camera raw sensor size, default 1280x800 (full
                         sensor, always y_start=0). The binned ROI modes
                         (640x200, 640x100) also work -- pass --y-start to
                         bracket the beam for those.
    --y-start N          real sensor row to center the ROI window on, for
                         the binned modes only (see beam_position_streamer.py
                         for why this can't be inherited from elsewhere).
    --port PORT          serial port for the FTA controller Nucleo. Default:
                         auto-detect by USB description (ST-Link VCP).
    --frames-per-point N camera frames averaged per grid point after each SD
                         trigger, default 3 -- reduces centroid noise per
                         calibration point. Separate from (and simpler than)
                         the multi-CAMERA averaging idea noted for later.
    --out PATH           where to save the raw sweep + fitted matrices
                         (.npz). Default: results/fta_calibration_<UTC
                         timestamp>.npz
    --dry-run            skip opening the serial link / sending grid_scan
                         entirely -- just capture+detect a few times from
                         whatever's in front of the camera right now, to
                         validate the capture/detection pipeline without the
                         Nucleo connected. Nothing moves, so no fit is
                         attempted -- this only proves the plumbing works.

Emergency stop: sends a single '!' byte (main.c's ISR-level hard-stop,
bypasses the line parser entirely -- see HAL_UART_RxCpltCallback) on Ctrl-C
or any other abort, so an interrupted run doesn't leave the FTA mid-scan.

Not yet run against real hardware -- written from the firmware source in
ucsbdeepspace/7-element-array (lock_in_2), not yet validated live.
"""
import argparse
import re
import time
from datetime import datetime, timezone

import cv2
import numpy as np
from picamera2 import Picamera2

from roi_set_selection import get_max_y_start, set_roi_y_start

EXPOSURE_US = 1500
ANALOGUE_GAIN = 4.0
CAM_INDEX = 0

FRAME_DURATION_US_BY_SIZE = {
    (1280, 800): 6000,
    (640, 200): 1800,
    (640, 100): 1050,
}
V_BIN_RATIO_BY_SIZE = {
    (1280, 800): 1,
    (640, 200): 2,
    (640, 100): 2,
}

MIN_BLOB_AREA_PX = 15
CONTRAST_CONFIDENCE_K = 5.0
MASK_THRESH_K = 3.0

FTA_BAUD = 460800
STALL_TIMEOUT_S = 30  # abort if no serial line arrives for this long
SD_LINE_RE = re.compile(r"^SD\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s*$")


def find_beam_blob(frame):
    """Duplicated from camera_view_tool.py/beam_position_streamer.py --
    see the latter's module docstring for why this isn't imported."""
    median = float(np.median(frame))
    std = float(frame.std())
    peak = float(frame.max())
    if std == 0 or (peak - median) < CONTRAST_CONFIDENCE_K * std:
        return None

    mask = (frame >= median + MASK_THRESH_K * std).astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    peak_y, peak_x = np.unravel_index(np.argmax(frame), frame.shape)
    blob = next((c for c in contours
                 if cv2.pointPolygonTest(c, (int(peak_x), int(peak_y)), False) >= 0),
                max(contours, key=cv2.contourArea))
    if cv2.contourArea(blob) < MIN_BLOB_AREA_PX:
        return None

    _, radius = cv2.minEnclosingCircle(blob)
    bx, by, bw, bh = cv2.boundingRect(blob)
    blob_mask = np.zeros((bh, bw), dtype=np.uint8)
    cv2.drawContours(blob_mask, [blob], -1, 255, thickness=cv2.FILLED, offset=(-bx, -by))
    roi = frame[by:by+bh, bx:bx+bw].astype(np.float64) * (blob_mask > 0)
    total = roi.sum()
    if total <= 0:
        return None
    ys_idx, xs_idx = np.indices((bh, bw), dtype=np.float64)
    cx = bx + float((roi * xs_idx).sum() / total)
    cy = by + float((roi * ys_idx).sum() / total)
    return cx, cy, radius


def find_fta_port():
    """Auto-detect the Nucleo's USB-serial port by USB description -- same
    tags nucleo_serial_monitor.py uses on the laptop side for the ST-Link
    VCP. This project's Nucleos all enumerate as ST-Link virtual COM ports
    regardless of which firmware they're running."""
    from serial.tools import list_ports
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    if not candidates:
        return None
    return candidates[0].device


def capture_centroid(cam, v_bin, y_start, n_frames):
    """Capture n_frames after a grid point's settle trigger and average the
    successful centroid detections. Returns (cx, cy, n_valid) in REAL
    absolute sensor pixels, or None if no frame detected a beam."""
    xs, ys = [], []
    for _ in range(n_frames):
        frame = cam.capture_array("raw").view(np.uint16)
        found = find_beam_blob(frame)
        if found is not None:
            cx, cy, _radius = found
            xs.append(cx * v_bin)
            ys.append(y_start + cy * v_bin)
    if not xs:
        return None
    return float(np.mean(xs)), float(np.mean(ys)), len(xs)


def apply_y_start(target):
    """Same retry-verify pattern as beam_position_streamer.py -- cam.start()
    can return before the driver settles into a freshly-selected mode."""
    max_y_start = get_max_y_start(CAM_INDEX)
    expected = max(0, min(target, max_y_start))
    expected -= expected % 4
    landed = None
    for _ in range(10):
        landed = set_roi_y_start(CAM_INDEX, target)
        if landed == expected:
            return landed
        time.sleep(0.05)
    print(f"WARNING: y_start did not settle at {target}, landed at {landed}")
    return landed


def fit_affine_3x3(dac_xy, cxy):
    """Least-squares fit of cxy = M @ [dac_x, dac_y, 1] for a 3x3 M with a
    fixed [0,0,1] bottom row (homogeneous-coordinate affine transform: a 2x2
    gain block + a 2x1 offset). Fits the FORWARD model (DAC -> centroid)
    because DAC setpoints are commanded exactly (noise-free independent
    variable) while centroids are noisy measurements -- standard
    least-squares assumes the noise lives on the dependent side.
    Returns (M, rms_residual_px)."""
    n = dac_xy.shape[0]
    design = np.column_stack([dac_xy, np.ones(n)])              # (n, 3)
    coeffs, _, _, _ = np.linalg.lstsq(design, cxy, rcond=None)  # (3, 2)
    M = np.eye(3)
    M[:2, :] = coeffs.T
    predicted = design @ coeffs
    residuals = cxy - predicted
    rms = float(np.sqrt(np.mean(np.sum(residuals ** 2, axis=1))))
    return M, rms


def run_dry_run(cam, v_bin, y_start, frames_per_point):
    print("--dry-run: no serial link, no real sweep. Capturing a few times "
          "from whatever's in front of the camera right now, just to "
          "validate the capture/detection pipeline. Nothing moves, so no "
          "fit is attempted.")
    for i in range(5):
        result = capture_centroid(cam, v_bin, y_start, frames_per_point)
        if result is None:
            print(f"  [{i}] NO BEAM DETECTED")
        else:
            cx, cy, n_valid = result
            print(f"  [{i}] centroid=({cx:.2f}, {cy:.2f}) "
                  f"({n_valid}/{frames_per_point} frames)")
        time.sleep(0.5)


def run_sweep(cam, v_bin, y_start, args):
    import serial

    port = args.port or find_fta_port()
    if port is None:
        print("No ST-Link serial port found -- pass --port explicitly, or "
              "check the Nucleo's USB cable is connected to this Pi.")
        raise SystemExit(1)
    print(f"Connecting to FTA controller on {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=1)
    time.sleep(2)  # let the Nucleo's USB-serial enumerate/settle
    ser.reset_input_buffer()

    records = []
    try:
        if args.grid_step is not None:
            ser.write(f"set_grid_step_size {args.grid_step}\n".encode("ascii"))
            print(f"< {ser.readline().decode(errors='replace').strip()}")

        cmd = f"grid_scan {args.x1} {args.y1} {args.x2} {args.y2}\n"
        print(f"Sending: {cmd.strip()}")
        ser.write(cmd.encode("ascii"))

        t_start = time.monotonic()
        last_line_time = t_start
        while True:
            line = ser.readline().decode(errors="replace").strip()
            now = time.monotonic()
            if not line:
                if now - last_line_time > STALL_TIMEOUT_S:
                    print(f"WARNING: no serial line in {STALL_TIMEOUT_S}s -- "
                          "aborting, check the link/firmware.")
                    break
                continue
            last_line_time = now
            print(f"< {line}")
            if line.startswith("ERR"):
                print("Firmware reported an error -- aborting.")
                break
            if line == "scan_complete":
                break

            match = SD_LINE_RE.match(line)
            if not match:
                continue
            adc, dac_x, dac_y = (int(g) for g in match.groups())
            result = capture_centroid(cam, v_bin, y_start, args.frames_per_point)
            if result is None:
                print(f"  dac=({dac_x},{dac_y}) NO BEAM DETECTED -- skipped")
                continue
            cx, cy, n_valid = result
            print(f"  dac=({dac_x},{dac_y}) adc={adc} -> "
                  f"centroid=({cx:.2f},{cy:.2f}) ({n_valid}/{args.frames_per_point})")
            records.append((dac_x, dac_y, adc, cx, cy))
            if len(records) % 10 == 0:
                elapsed = now - t_start
                print(f"  ... {len(records)} points, {elapsed:.1f}s elapsed")
    except KeyboardInterrupt:
        print("\nInterrupted -- sending emergency stop to firmware.")
    finally:
        try:
            ser.write(b"!")  # ISR-level hard stop, bypasses the line parser
        except Exception as e:
            print(f"WARNING: couldn't send emergency stop: {e}")
        ser.close()

    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("x1", type=int)
    parser.add_argument("y1", type=int)
    parser.add_argument("x2", type=int)
    parser.add_argument("y2", type=int)
    parser.add_argument("--grid-step", type=int, default=None)
    parser.add_argument("--raw-size", default="1280x800")
    parser.add_argument("--y-start", type=int, default=None)
    parser.add_argument("--port", default=None)
    parser.add_argument("--frames-per-point", type=int, default=3)
    parser.add_argument("--out", default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    for name, v in [("x1", args.x1), ("y1", args.y1), ("x2", args.x2), ("y2", args.y2)]:
        if not (0 <= v <= 4095):
            print(f"WARNING: {name}={v} outside 0-4095 -- firmware will clamp it.")

    w, h = args.raw_size.lower().split("x")
    raw_size = (int(w), int(h))
    if raw_size not in FRAME_DURATION_US_BY_SIZE:
        print(f"Unsupported size {raw_size}. Supported: {list(FRAME_DURATION_US_BY_SIZE)}")
        raise SystemExit(1)
    v_bin = V_BIN_RATIO_BY_SIZE[raw_size]

    cam = Picamera2(CAM_INDEX)
    config = cam.create_video_configuration(raw={"size": raw_size, "format": "R8"}, buffer_count=2)
    cam.configure(config)
    cam.start()
    frame_duration_us = FRAME_DURATION_US_BY_SIZE[raw_size]
    cam.set_controls({
        "FrameDurationLimits": (frame_duration_us, frame_duration_us),
        "AeEnable": False,
        "NoiseReductionMode": 0,
        "ExposureTime": EXPOSURE_US,
        "AnalogueGain": ANALOGUE_GAIN,
    })

    if raw_size == (1280, 800):
        y_start = 0
    elif args.y_start is not None:
        y_start = apply_y_start(args.y_start)
    else:
        y_start = 0
        print("WARNING: no --y-start given for a binned ROI mode -- see "
              "beam_position_streamer.py's docstring for why this matters.")

    try:
        if args.dry_run:
            run_dry_run(cam, v_bin, y_start, args.frames_per_point)
            return
        records = run_sweep(cam, v_bin, y_start, args)
    finally:
        cam.stop()
        cam.close()

    if len(records) < 3:
        print(f"Only {len(records)} usable points -- need at least 3 to fit "
              "a 3x3 affine (2x2 gain + offset has 6 free parameters, and "
              "each point only gives 2 equations). Not fitting.")
        return

    dac_xy = np.array([[r[0], r[1]] for r in records], dtype=np.float64)
    adc = np.array([r[2] for r in records], dtype=np.int64)
    cxy = np.array([[r[3], r[4]] for r in records], dtype=np.float64)
    M, rms = fit_affine_3x3(dac_xy, cxy)

    det2x2 = np.linalg.det(M[:2, :2])
    print(f"\n{len(records)} points fit. RMS residual: {rms:.2f} px. "
          f"2x2 gain block determinant: {det2x2:.4g}")
    if abs(det2x2) < 1e-6:
        print("WARNING: gain block is near-singular -- the two DAC axes "
              "don't independently move the centroid in two directions. "
              "Inverse map (centroid -> DAC) will be unreliable.")
    if rms > 5.0:
        print("WARNING: RMS residual is large for a linear fit -- check for "
              "nonlinearity, actuator hysteresis, or a bad grid point before "
              "trusting this calibration.")

    M_inv = np.linalg.inv(M)
    print("M (DAC -> centroid px):")
    print(M)
    print("M_inv (centroid px -> DAC):")
    print(M_inv)

    out_path = args.out
    if out_path is None:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        out_path = f"results/fta_calibration_{ts}.npz"
    np.savez(
        out_path,
        dac_x=dac_xy[:, 0], dac_y=dac_xy[:, 1],
        cx=cxy[:, 0], cy=cxy[:, 1],
        adc=adc, M=M, M_inv=M_inv, rms_residual_px=rms,
    )
    print(f"Saved to {out_path}")


if __name__ == "__main__":
    main()
