#!/usr/bin/env python3
"""
Characterizes the FTA's actual DYNAMIC response (rise time, settling time,
overshoot) to a step change in DAC setpoint -- something the static
DAC->centroid calibration (fta_calibration.py) cannot tell us, since that
only measures where the beam ends up AFTER settling, not how fast it gets
there or whether it overshoots/rings on the way. This data is needed before
picking real Kp/Ki gains for the PI control law discussed in CLAUDE.md --
gains that look fine on paper can ring or go unstable if the actuator's own
mechanical response is slower/more oscillatory than assumed.

Commands a step in ONE axis (x or y) via the "FTA Controller" Nucleo's
set_x/set_y serial command, then captures camera frames at full speed (no
artificial throttle, same approach as beam_position_streamer.py) spanning
the step, logging (timestamp, cx, cy) via find_beam_blob for every frame
where a beam was detected. Saves the raw time series to results/ and
prints basic step-response metrics computed from it:

  rise time      -- time from step onset to first crossing of 90% of the
                    (final - baseline) delta, using the STEPPED axis's
                    pixel coordinate as the primary trace (10% crossing to
                    90% crossing, standard convention)
  overshoot       -- max excursion past the final value, as a % of the
                    step size
  settling time   -- time after step onset when the signal FIRST enters
                    AND STAYS within --settle-tol-px of the final value
                    through the end of the recorded window. Caveat: this
                    can only confirm settling within --post-s of data --
                    if the true settling time is close to or longer than
                    --post-s, this will underreport it or show "not
                    settled" instead of a real number.

Both cx and cy are logged regardless of which axis was stepped -- this
also surfaces cross-axis coupling (does stepping DAC-x visibly perturb
cy too?), which is useful information on its own, separate from the
primary-axis metrics above.

Usage:
  python3 fta_step_response_test.py --step-to N [--axis x|y] [--step-from N]
      [--raw-size WxH] [--y-start N] [--pre-s SEC] [--post-s SEC]
      [--settle-tol-px PX] [--port PORT] [--out PATH]

    --axis            which DAC channel to step, default x
    --step-from N      starting DAC setpoint, commanded and allowed to
                       settle before recording begins. Default: read the
                       FTA's own current position via get_status (no
                       assumption, matches fta_calibration.py's pattern).
    --step-to N        DAC setpoint to step TO -- required, no default,
                       same reasoning as fta_calibration.py's grid corners:
                       a step size is a deliberate per-setup choice.
    --raw-size WxH     camera raw sensor size, default 1280x800 (full
                       sensor, always y_start=0). Binned ROI modes also
                       work -- pass --y-start to bracket the beam.
    --y-start N        real sensor row for binned ROI modes.
    --pre-s SEC        seconds of baseline recorded BEFORE the step,
                       default 0.2
    --post-s SEC       seconds recorded AFTER the step, default 1.0 --
                       widen this if the actuator might settle slower
                       than that (unknown until this test has been run at
                       least once).
    --settle-tol-px PX pixel tolerance band for "settled", default 2.0
                       (= 6.0um at this sensor's 3.0um/px pitch, confirmed
                       via Picamera2's own UnitCellSize, not a datasheet
                       guess -- see MICRONS_PER_PIXEL)
    --port PORT        FTA controller serial port, default auto-detect
    --out PATH         where to save the raw (t, cx, cy) time series --
                       default results/fta_step_response_<axis>_<UTC
                       timestamp>.npz

Not yet run against real hardware with the FTA actually driving optics --
only the serial link itself has been validated so far
(fta_serial_latency_test.py), not the physical actuator response.
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

# OV9281 physical pixel pitch, confirmed live via Picamera2(0).camera_properties
# -- "UnitCellSize": (3000, 3000) nanometers -- not a datasheet guess. This is
# the real-sensor pitch: since the pixel coordinates this script logs are
# already scaled by v_bin (real absolute sensor pixels, same convention as
# beam_position_streamer.py/fta_calibration.py), multiplying by this constant
# directly gives real microns of centroid displacement regardless of which
# raw_size/binning mode was used -- no separate binning correction needed here.
MICRONS_PER_PIXEL = 3.0

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

FTA_BAUD = 115200  # camera_centroid_receiver's USART2 rate (unchanged from
# the original heartbeat-only firmware) -- NOT the old "FTA Controller"'s
# 460800. Get this wrong and every reply is baud-mismatch garbage, not a
# clean protocol error (same signature CLAUDE.md already documented once,
# 2026-07-23, when the two firmwares' baud rates were mixed up).
# camera_centroid_receiver's phase-1 firmware (nucleo_firmware/, 2026-08-04),
# not the old "FTA Controller" -- reply format changed from positional CSV
# ("status:x,y,...") to keyed text. dac_x/dac_y (the last commanded DAC
# setpoint) is this firmware's equivalent of "FTA Controller"'s reported
# x/y position; tel_x/tel_y is a different thing (relayed camera telemetry,
# not the actuator setpoint) and must not be used here.
STATUS_RE = re.compile(r"dac_x=(-?\d+)\s+dac_y=(-?\d+)")


def find_beam_blob(frame):
    """Duplicated from camera_view_tool.py/beam_position_streamer.py/
    fta_calibration.py -- see beam_position_streamer.py's module docstring
    for why this isn't imported."""
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
    tags used in fta_calibration.py/fta_serial_latency_test.py."""
    from serial.tools import list_ports
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    if not candidates:
        return None
    return candidates[0].device


def get_current_position(ser):
    """Retries the whole request a few times, not just the read -- bench
    testing this firmware (2026-08-04) found the VCP link occasionally
    drops a character out of a command when it lands during the Pi's
    high-rate I2C telemetry stream (I2C1 is a higher NVIC priority than
    USART2), corrupting e.g. "get_status" into "ge_status". The firmware
    always replies ERR to a corrupted line rather than hanging, so a
    fresh attempt is enough to recover -- no firmware change needed for
    this, but a single-shot request isn't reliable enough to build on."""
    for _ in range(5):
        ser.reset_input_buffer()
        ser.write(b"get_status\n")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            m = STATUS_RE.search(raw.decode(errors="replace").strip())
            if m:
                return int(m.group(1)), int(m.group(2))
    raise RuntimeError("No get_status reply after 5 attempts -- check the serial link/firmware.")


def apply_y_start(target):
    """Same retry-verify pattern as beam_position_streamer.py/
    fta_calibration.py -- cam.start() can return before the driver settles
    into a freshly-selected mode."""
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


def analyze_step(t, primary, t_step, settle_tol_px):
    """t, primary are numpy arrays of (timestamp, pixel coordinate) for the
    stepped axis's own pixel trace. Returns a dict of metrics, or None if
    there isn't enough data on either side of the step to compute them."""
    pre_mask = t < t_step
    post_mask = ~pre_mask
    if pre_mask.sum() < 3 or post_mask.sum() < 3:
        return None

    baseline = float(np.mean(primary[pre_mask]))
    tail_mask = post_mask & (t > t[-1] - 0.2 * (t[-1] - t_step))
    final = float(np.mean(primary[tail_mask])) if tail_mask.sum() >= 3 else float(primary[post_mask][-1])

    delta = final - baseline
    if delta == 0:
        return {"baseline": baseline, "final": final, "delta": delta,
                "rise_time_s": None, "overshoot_pct": None, "settling_time_s": None,
                "note": "no net change detected -- actuator may not be moving the beam at all"}

    post_t = t[post_mask] - t_step
    post_v = primary[post_mask]
    frac = (post_v - baseline) / delta  # 0 at baseline, 1 at final, could exceed 1 (overshoot)

    def first_crossing(target_frac):
        idx = np.where(frac >= target_frac)[0] if delta > 0 else np.where(frac <= target_frac)[0]
        return post_t[idx[0]] if len(idx) else None

    t10 = first_crossing(0.10)
    t90 = first_crossing(0.90)
    rise_time = (t90 - t10) if (t10 is not None and t90 is not None and t90 >= t10) else None

    max_frac = float(np.max(frac)) if delta > 0 else float(-np.min(frac))
    overshoot_pct = max(0.0, (max_frac - 1.0) * 100.0)

    within_tol = np.abs(post_v - final) <= settle_tol_px
    settling_time = None
    for i in range(len(post_t)):
        if np.all(within_tol[i:]):
            settling_time = post_t[i]
            break

    return {
        "baseline": baseline, "final": final, "delta": delta,
        "rise_time_s": rise_time, "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "note": None if settling_time is not None else
                f"never stayed within {settle_tol_px}px of final for the rest "
                "of the recorded window -- widen --post-s to get a real number",
    }


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--step-to", type=int, required=True)
    parser.add_argument("--axis", choices=["x", "y"], default="x")
    parser.add_argument("--step-from", type=int, default=None)
    parser.add_argument("--raw-size", default="1280x800")
    parser.add_argument("--y-start", type=int, default=None)
    parser.add_argument("--pre-s", type=float, default=0.2)
    parser.add_argument("--post-s", type=float, default=1.0)
    parser.add_argument("--settle-tol-px", type=float, default=2.0)
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    import serial

    port = args.port or find_fta_port()
    if port is None:
        print("No ST-Link serial port found -- pass --port explicitly, or "
              "check the Nucleo's USB cable is connected to this Pi.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    cur_x, cur_y = get_current_position(ser)
    step_from = args.step_from if args.step_from is not None else (cur_x if args.axis == "x" else cur_y)
    print(f"Current position: x={cur_x} y={cur_y}. Stepping axis '{args.axis}' "
          f"from {step_from} to {args.step_to}.")

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

    records = []  # (t, cx, cy)
    try:
        # Settle at step_from before recording anything.
        ser.write(f"set_{args.axis} {step_from}\n".encode("ascii"))
        time.sleep(0.5)

        t0 = time.monotonic()
        t_step = None
        while True:
            now = time.monotonic() - t0
            if t_step is None and now >= args.pre_s:
                ser.write(f"set_{args.axis} {args.step_to}\n".encode("ascii"))
                t_step = now
            if t_step is not None and now - t_step >= args.post_s:
                break

            frame = cam.capture_array("raw").view(np.uint16)
            found = find_beam_blob(frame)
            if found is not None:
                cx, cy, _radius = found
                records.append((now, cx * v_bin, y_start + cy * v_bin))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cam.stop()
        cam.close()
        ser.close()

    if len(records) < 6:
        print(f"Only {len(records)} usable frames -- not enough to analyze. "
              "Check the beam is actually visible/detected throughout.")
        return

    t = np.array([r[0] for r in records])
    cx = np.array([r[1] for r in records])
    cy = np.array([r[2] for r in records])

    # Don't assume DAC-x drives pixel-x -- the actuator can be rotated
    # relative to the camera (confirmed live on this rig: DAC-x mostly
    # drives pixel-y, not pixel-x). Auto-pick whichever pixel axis actually
    # moved more between the pre- and post-step windows as "primary" for
    # the rise/overshoot/settling analysis below.
    pre_mask, post_mask = t < t_step, t >= t_step
    delta_cx = abs(cx[post_mask].mean() - cx[pre_mask].mean()) if pre_mask.any() and post_mask.any() else 0.0
    delta_cy = abs(cy[post_mask].mean() - cy[pre_mask].mean()) if pre_mask.any() and post_mask.any() else 0.0
    if delta_cy > delta_cx:
        primary, primary_name = cy, "cy"
    else:
        primary, primary_name = cx, "cx"
    print(f"Dominant response axis: {primary_name} (|delta cx|={delta_cx:.1f}px, "
          f"|delta cy|={delta_cy:.1f}px) -- analyzing {primary_name}.")

    print(f"Captured {len(records)} frames ({t[-1] - t[0]:.3f}s span, "
          f"~{len(records) / (t[-1] - t[0]):.0f}fps average).")

    metrics = analyze_step(t, primary, t_step, args.settle_tol_px)
    if metrics is None:
        print("Not enough frames before/after the step to compute metrics.")
    else:
        um = MICRONS_PER_PIXEL
        print(f"baseline={metrics['baseline']:.2f}px ({metrics['baseline']*um:.1f}um)  "
              f"final={metrics['final']:.2f}px ({metrics['final']*um:.1f}um)  "
              f"delta={metrics['delta']:.2f}px ({metrics['delta']*um:.1f}um)")
        if metrics["rise_time_s"] is not None:
            print(f"rise time (10%-90%): {metrics['rise_time_s'] * 1000:.1f}ms")
        else:
            print("rise time: could not be determined (check delta/noise)")
        if metrics["overshoot_pct"] is not None:
            print(f"overshoot: {metrics['overshoot_pct']:.1f}%")
        if metrics["settling_time_s"] is not None:
            print(f"settling time (within {args.settle_tol_px}px / "
                  f"{args.settle_tol_px*um:.1f}um): "
                  f"{metrics['settling_time_s'] * 1000:.1f}ms")
        if metrics["note"]:
            print(f"NOTE: {metrics['note']}")

    out_path = args.out
    if out_path is None:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        out_path = f"results/fta_step_response_{args.axis}_{ts}.npz"
    np.savez(out_path, t=t, cx=cx, cy=cy, t_step=t_step,
             step_from=step_from, step_to=args.step_to, axis=args.axis)
    print(f"Saved raw time series to {out_path}")


if __name__ == "__main__":
    main()
