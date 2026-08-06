#!/usr/bin/env python3
"""
Sweeps the FTA over a grid of DAC setpoints and records the relayed beam
centroid at each point, then fits a 3x3 affine matrix (homogeneous 2x2 gain
+ offset) mapping DAC setpoint -> centroid pixel, plus its inverse
(centroid -> DAC setpoint) for later closed-loop use.

Replaces fta_calibration.py for this firmware. That script drove the sweep
via the OLD "FTA Controller" firmware's `grid_scan x1 y1 x2 y2` command
(smooth, backlash-aware microstep travel, one `SD <adc> <x> <y>` line per
sampled point) and captured via Picamera2 directly on the Pi -- neither
exists here: camera_centroid_receiver has no grid_scan (per the
2026-08-04 architecture decision, dropped rather than ported -- see
CLAUDE.md), and under the v2 architecture the Pi is telemetry-only, so
this drives the sweep itself with plain `set_x`/`set_y` jumps and settle
waits, reading position from the Nucleo's I2C-relay print over the VCP --
same laptop-only approach validated in fta_step_response_test_vcp.py and
fta_sine_response_test_vcp.py. No Pi access needed, as long as something
there (camera_view_tool.py or beam_position_streamer.py) is already
streaming.

**Not yet tested for hysteresis**: CLAUDE.md flags that this actuator's
position-dependent hysteresis (different reading approaching a point from
opposite directions) hasn't been characterized. Grid points are swept in
serpentine (boustrophedon) row order, which keeps travel direction
consistent within a row but reverses between rows -- if hysteresis turns
out to matter, that reversal would show up as a seam between rows, worth
checking for in the residuals if the RMS comes back surprisingly large.

Usage:
  python3 fta_calibration_vcp.py X1 Y1 X2 Y2 [--grid-step N]
      [--settle-s SEC] [--capture-s SEC] [--port PORT] [--out PATH]

    X1 Y1 X2 Y2    DAC-count grid corners (firmware clamps to [95, 4000])
                   -- required, no default, a safe sweep range is a
                   deliberate per-setup choice.
    --grid-step N  DAC counts between grid samples, default 300 (coarse,
                   for a fast first pass -- narrow it once a rough
                   calibration confirms the safe/valid range).
    --settle-s SEC  wait after each set_x/set_y before capturing, default
                   0.3 -- comfortably above the largest small-step settling
                   time measured in the step-response tests (469ms was the
                   outlier; most were under 250ms), short of the ~1s
                   large-step settling times (grid steps stay in the
                   small-step regime by design, matching CLAUDE.md's
                   guidance for why grid_scan wasn't needed here).
    --capture-s SEC  how long to average relayed telemetry samples over
                   after settling, default 0.15 (~25-30 samples at the
                   ~170-190/s relay rate).
    --port PORT    Nucleo VCP serial port, default auto-detect.
    --out PATH     where to save the raw sweep + fitted matrices --
                   default results/fta_calibration_vcp_<UTC timestamp>.npz
"""
import argparse
import re
import time
from datetime import datetime, timezone

import numpy as np

FTA_BAUD = 115200  # camera_centroid_receiver's USART2 rate, NOT the old
                    # "FTA Controller"'s 460800.

TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+pkts=(\d+)\s+errs=(\d+)$")

STATUS_FIELD_RE = {
    "dac_x": re.compile(r"dac_x=(-?\d+)"),
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
}


def find_fta_port():
    from serial.tools import list_ports
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    if not candidates:
        return None
    return candidates[0].device


def get_status(ser):
    """Returns (dac_x, dac_y, amp, tel_age_ms). Retries the whole request a
    few times -- the VCP can still occasionally drop a character out of a
    command (rare since the 2026-08-06 NVIC priority fix, not impossible);
    the firmware always replies ERR to a corrupted line rather than
    hanging, so a fresh attempt recovers."""
    for _ in range(5):
        ser.reset_input_buffer()
        ser.write(b"get_status\n")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw or not raw.startswith(b"STATUS"):
                continue
            line = raw.decode(errors="replace").strip()
            matches = {k: rx.search(line) for k, rx in STATUS_FIELD_RE.items()}
            if all(matches.values()):
                return (int(matches["dac_x"].group(1)), int(matches["dac_y"].group(1)),
                        int(matches["amp"].group(1)), int(matches["tel_age_ms"].group(1)))
    raise RuntimeError("No get_status reply after 5 attempts -- check the serial link/firmware.")


def capture_centroid(ser, capture_s):
    """Averages relayed telemetry samples (status bit0 set, i.e. a
    confident detection) received over the next capture_s seconds.
    Returns (cx, cy, n_valid), or None if nothing usable arrived.
    reset_input_buffer() first discards whatever accumulated during the
    preceding settle sleep, unread -- otherwise the first read here would
    burst through stale samples from before settling finished (same class
    of bug fixed in fta_sine_response_test_vcp.py's reader thread, simpler
    to just avoid here since this function doesn't need precise timing,
    only a clean averaging window)."""
    ser.reset_input_buffer()
    xs, ys = [], []
    deadline = time.monotonic() + capture_s
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        m = TELEMETRY_RE.match(raw.decode(errors="replace").strip())
        if not m or not (int(m.group(2)) & 1):
            continue
        xs.append(float(m.group(3)))
        ys.append(float(m.group(4)))
    if not xs:
        return None
    return float(np.mean(xs)), float(np.mean(ys)), len(xs)


def fit_affine_3x3(dac_xy, cxy):
    """Identical to fta_calibration.py's fit_affine_3x3 -- least-squares
    fit of cxy = M @ [dac_x, dac_y, 1] for a 3x3 M with a fixed [0,0,1]
    bottom row. Fits the FORWARD model (DAC -> centroid) because DAC
    setpoints are commanded exactly while centroids are noisy
    measurements. Returns (M, rms_residual_px)."""
    n = dac_xy.shape[0]
    design = np.column_stack([dac_xy, np.ones(n)])
    coeffs, _, _, _ = np.linalg.lstsq(design, cxy, rcond=None)
    M = np.eye(3)
    M[:2, :] = coeffs.T
    predicted = design @ coeffs
    residuals = cxy - predicted
    rms = float(np.sqrt(np.mean(np.sum(residuals ** 2, axis=1))))
    return M, rms


def serpentine_grid(x1, y1, x2, y2, step):
    """Row-major grid points, alternating x-direction each row (a
    boustrophedon sweep) -- keeps travel continuous within a row and
    limits any hysteresis asymmetry to row boundaries rather than every
    single point."""
    lo_x, hi_x = min(x1, x2), max(x1, x2)
    lo_y, hi_y = min(y1, y2), max(y1, y2)
    xs = list(range(lo_x, hi_x + 1, step)) or [lo_x]
    ys = list(range(lo_y, hi_y + 1, step)) or [lo_y]
    points = []
    for i, y in enumerate(ys):
        row_xs = xs if i % 2 == 0 else list(reversed(xs))
        for x in row_xs:
            points.append((x, y))
    return points


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("x1", type=int)
    parser.add_argument("y1", type=int)
    parser.add_argument("x2", type=int)
    parser.add_argument("y2", type=int)
    parser.add_argument("--grid-step", type=int, default=300)
    parser.add_argument("--settle-s", type=float, default=0.3)
    parser.add_argument("--capture-s", type=float, default=0.15)
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    for name, v in [("x1", args.x1), ("y1", args.y1), ("x2", args.x2), ("y2", args.y2)]:
        if not (95 <= v <= 4000):
            print(f"WARNING: {name}={v} outside [95, 4000] -- firmware will clamp it.")

    import serial

    port = args.port or find_fta_port()
    if port is None:
        print("No ST-Link serial port found -- pass --port explicitly, or "
              "check the Nucleo's USB cable is connected to this machine.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    dac_x0, dac_y0, amp, tel_age_ms = get_status(ser)
    if tel_age_ms > 500:
        print(f"ERR: last relayed I2C telemetry is {tel_age_ms}ms old -- nothing "
              "appears to be streaming from the Pi. Start camera_view_tool.py or "
              "beam_position_streamer.py there first, then retry.")
        raise SystemExit(1)
    print(f"Telemetry is live (last packet {tel_age_ms}ms old). "
          f"Current DAC position: x={dac_x0} y={dac_y0}, amp={'on' if amp else 'off'}.")

    amp_was_enabled = bool(amp)
    if not amp_was_enabled:
        print("Amp is currently disabled -- enabling it for this sweep "
              "(will be restored to disabled afterward).")
        ser.write(b"amp_enable\n")
        time.sleep(0.1)
        _, _, amp_confirmed, _ = get_status(ser)
        if not amp_confirmed:
            print("ERR: sent amp_enable but get_status still reports amp=off -- "
                  "aborting rather than sweeping with the amp off.")
            ser.close()
            raise SystemExit(1)

    points = serpentine_grid(args.x1, args.y1, args.x2, args.y2, args.grid_step)
    print(f"Sweeping {len(points)} grid points, step={args.grid_step}, "
          f"settle={args.settle_s}s, capture={args.capture_s}s "
          f"(~{len(points) * (args.settle_s + args.capture_s):.0f}s estimated).")

    records = []
    n_skipped = 0
    try:
        t_start = time.monotonic()
        for i, (dac_x, dac_y) in enumerate(points):
            ser.write(f"set_x {dac_x}\n".encode("ascii"))
            ser.write(f"set_y {dac_y}\n".encode("ascii"))
            time.sleep(args.settle_s)
            result = capture_centroid(ser, args.capture_s)
            if result is None:
                print(f"  [{i+1}/{len(points)}] dac=({dac_x},{dac_y}) NO BEAM DETECTED -- skipped")
                n_skipped += 1
                continue
            cx, cy, n_valid = result
            records.append((dac_x, dac_y, cx, cy))
            if (i + 1) % 10 == 0 or i == len(points) - 1:
                elapsed = time.monotonic() - t_start
                print(f"  [{i+1}/{len(points)}] dac=({dac_x},{dac_y}) -> "
                      f"centroid=({cx:.2f},{cy:.2f}) ({n_valid} samples) "
                      f"[{elapsed:.0f}s elapsed]")
    except KeyboardInterrupt:
        print("\nInterrupted -- sending emergency stop to firmware.")
        try:
            ser.write(b"!")
        except Exception as e:
            print(f"WARNING: couldn't send emergency stop: {e}")
    finally:
        try:
            ser.write(f"set_x {dac_x0}\n".encode("ascii"))
            ser.write(f"set_y {dac_y0}\n".encode("ascii"))
            time.sleep(0.1)
            if not amp_was_enabled:
                ser.write(b"amp_disable\n")
                time.sleep(0.1)
        except Exception as e:
            print(f"WARNING: couldn't restore idle state: {e}")
        ser.close()

    print(f"\n{len(records)}/{len(points)} points captured ({n_skipped} skipped, no beam detected).")
    if len(records) < 3:
        print("Fewer than 3 usable points -- need at least 3 to fit a 3x3 affine "
              "(2x2 gain + offset has 6 free parameters, each point gives 2 "
              "equations). Not fitting.")
        return

    dac_xy = np.array([[r[0], r[1]] for r in records], dtype=np.float64)
    cxy = np.array([[r[2], r[3]] for r in records], dtype=np.float64)
    M, rms = fit_affine_3x3(dac_xy, cxy)

    det2x2 = np.linalg.det(M[:2, :2])
    print(f"RMS residual: {rms:.2f}px. 2x2 gain block determinant: {det2x2:.4g}")
    if abs(det2x2) < 1e-6:
        print("WARNING: gain block is near-singular -- the two DAC axes don't "
              "independently move the centroid in two directions. Inverse map "
              "(centroid -> DAC) will be unreliable.")
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
        out_path = f"results/fta_calibration_vcp_{ts}.npz"
    np.savez(out_path, dac_x=dac_xy[:, 0], dac_y=dac_xy[:, 1],
              cx=cxy[:, 0], cy=cxy[:, 1], M=M, M_inv=M_inv, rms_residual_px=rms)
    print(f"Saved to {out_path}")


if __name__ == "__main__":
    main()
