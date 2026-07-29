#!/usr/bin/env python3
"""
Measures the REAL achieved loop rate of the actual planned closed-loop
pipeline -- capture frame -> find_beam_blob -> send position over the FTA
Controller's USB-serial link -- with everything except the PID math itself
(no gain computation, no calibration-matrix inverse, nothing actually
steered). This exists to answer a specific open question directly instead
of extrapolating it from two separately-measured numbers: CLAUDE.md's
camera-detection latency (~3.5-4ms mean, MODE_640_100_ROI) and
fta_serial_latency_test.py's serial round-trip latency (~1.3ms mean,
single-axis, wait-for-ack) were never combined into one real, running loop.
Naive addition suggests serial is affordable inside Phil's 10ms budget, but
that's an estimate, not a measurement -- and it's also the reason
nucleo_i2c_sender.py (I2C) exists at all, so this script's numbers are what
actually settle whether that I2C link is still needed for the real-time
control path, or whether serial alone comfortably closes the loop.

Each iteration: capture one raw frame, run find_beam_blob, and (depending on
--send-mode) send the FTA's OWN CURRENT position back to it over serial for
BOTH axes -- never a new/different setpoint, so nothing physically moves,
same non-destructive convention fta_serial_latency_test.py established. The
point is to pay the REAL wire cost of a 2-axis command each loop, not to
actually servo anything -- that's the "everything but the PID" framing:
this measures the pipeline's ceiling, a real PID would ride under it.

--send-mode:
  none            capture + detect only, no serial link opened at all. This
                   is the pure camera-side ceiling, measured with this exact
                   script/methodology (not just cited from an older test), so
                   it's an apples-to-apples baseline for the other two modes.
  wait_ack        (default) send_x then send_y, each a full write-then-wait-
                   for-ack round trip before the next axis is sent -- the
                   simple, synchronous way a first real control loop would
                   plausibly be written. This is the REALISTIC worst case:
                   two sequential round trips per loop, not one.
  fire_and_forget send_x and send_y back-to-back with zero waiting, no ack
                   read at all -- the theoretical ceiling if the real
                   controller never needs delivery confirmation. Also diffs
                   the firmware's own cmdq_stats drop counter across the run
                   to check nothing was silently lost at this rate, same
                   check fta_serial_latency_test.py's burst/sweep modes use.

On a lost-beam frame (find_beam_blob returns None), no send happens for that
iteration -- there's no PID state here to freeze/hold, so it's simply
skipped and counted separately, not sent as a bogus value.

Usage:
  python3 fta_closed_loop_fps_test.py [--send-mode none|wait_ack|fire_and_forget]
      [--raw-size WxH] [--y-start N] [--duration-s SEC] [--port PORT] [--out PATH]

Requires the Nucleo's USB connected directly to this Pi (for --send-mode
other than none) and a real beam visible to the camera. Not yet run against
real hardware -- written on a device with no camera/Nucleo attached, from
the same firmware/protocol assumptions as fta_calibration.py and
fta_serial_latency_test.py (which HAVE been validated live). One assumption
here is new and unverified: the set_y ack line is assumed to read
"y_center set to N", symmetric to fta_serial_latency_test.py's confirmed
set_x ack ("x_center set to N") -- if that's wrong, wait_ack mode will fail
LOUDLY (near-zero successful y round trips in the report), not silently
mismeasure, since a sample is only counted on an actual regex match.
"""
import argparse
import re
import statistics
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
STATUS_RE = re.compile(r"^status:(-?\d+),(-?\d+),(-?\d+),")
SET_X_ACK_RE = re.compile(r"^x_center set to (-?\d+)\s*$")
# Assumed symmetric to SET_X_ACK_RE -- see module docstring caveat.
SET_Y_ACK_RE = re.compile(r"^y_center set to (-?\d+)\s*$")
CMDQ_STATS_RE = re.compile(r"^cmdq depth=(\d+) dropped=(\d+)\s*$")


def find_beam_blob(frame):
    """Duplicated from camera_view_tool.py/beam_position_streamer.py/
    fta_calibration.py/fta_step_response_test.py -- see
    beam_position_streamer.py's module docstring for why this isn't
    imported. Mirror any confidence-gate constant changes here too."""
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
    ser.reset_input_buffer()
    ser.write(b"get_status\n")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        m = STATUS_RE.match(raw.decode(errors="replace").strip())
        if m:
            return int(m.group(2)), int(m.group(3))
    raise RuntimeError("No get_status reply -- check the serial link/firmware.")


def get_cmdq_stats(ser, retries=3):
    """Same retry pattern as fta_serial_latency_test.py -- a query right
    after a fast burst occasionally gets no reply within budget."""
    for attempt in range(retries):
        ser.reset_input_buffer()
        ser.write(b"cmdq_stats\n")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            m = CMDQ_STATS_RE.match(raw.decode(errors="replace").strip())
            if m:
                return int(m.group(1)), int(m.group(2))
        time.sleep(0.2)
    raise RuntimeError(f"No cmdq_stats reply after {retries} attempts")


def send_and_wait(ser, command, ack_re, budget_s=0.5):
    """One write-then-wait-for-matching-reply round trip. Returns elapsed
    seconds, or None on timeout/no match (skipped, not counted as a bogus
    fast/slow sample -- same convention as fta_serial_latency_test.py)."""
    t0 = time.monotonic()
    ser.write((command + "\n").encode("ascii"))
    while time.monotonic() - t0 < budget_s:
        raw = ser.readline()
        if not raw:
            continue
        if ack_re.match(raw.decode(errors="replace").strip()):
            return time.monotonic() - t0
    return None


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


def report_ms(label, samples_s, n_iterations):
    if not samples_s:
        print(f"{label}: no successful samples out of {n_iterations} iterations.")
        return
    ms = sorted(s * 1000.0 for s in samples_s)
    n = len(ms)

    def pct(p):
        idx = min(n - 1, int(round(p / 100 * (n - 1))))
        return ms[idx]

    print(f"{label}: n={n}/{n_iterations}  "
          f"min={ms[0]:.3f}ms  mean={statistics.mean(ms):.3f}ms  "
          f"median={statistics.median(ms):.3f}ms  "
          f"p95={pct(95):.3f}ms  p99={pct(99):.3f}ms  max={ms[-1]:.3f}ms  "
          f"(mean -> {1000.0 / statistics.mean(ms):.1f}Hz)")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--send-mode", choices=["none", "wait_ack", "fire_and_forget"],
                         default="wait_ack")
    parser.add_argument("--raw-size", default="640x100")
    parser.add_argument("--y-start", type=int, default=None)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--warmup-s", type=float, default=0.5,
                         help="Discard iterations before this many seconds have "
                              "elapsed, to let the camera/link settle.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    w, h = args.raw_size.lower().split("x")
    raw_size = (int(w), int(h))
    if raw_size not in FRAME_DURATION_US_BY_SIZE:
        print(f"Unsupported size {raw_size}. Supported: {list(FRAME_DURATION_US_BY_SIZE)}")
        raise SystemExit(1)
    v_bin = V_BIN_RATIO_BY_SIZE[raw_size]

    ser = None
    cur_x = cur_y = None
    dropped_before = dropped_after = None
    if args.send_mode != "none":
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

        cur_x, cur_y = get_current_position(ser)
        print(f"Current position x={cur_x} y={cur_y} -- re-sending this SAME "
              f"position every loop (nothing will actually move).")
        if args.send_mode == "fire_and_forget":
            _, dropped_before = get_cmdq_stats(ser)

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

    capture_s, detect_s, send_s, total_s = [], [], [], []
    n_beam_found = n_sent = n_iterations = 0
    t_run_start = time.monotonic()
    t_prev_loop_start = None

    try:
        while True:
            t_loop_start = time.monotonic()
            elapsed = t_loop_start - t_run_start
            if elapsed >= args.warmup_s + args.duration_s:
                break
            warmed_up = elapsed >= args.warmup_s

            t0 = time.monotonic()
            frame = cam.capture_array("raw").view(np.uint16)
            t1 = time.monotonic()
            found = find_beam_blob(frame)
            t2 = time.monotonic()

            sent_this_iter = False
            if found is not None and args.send_mode != "none":
                if args.send_mode == "wait_ack":
                    dt_x = send_and_wait(ser, f"set_x {cur_x}", SET_X_ACK_RE)
                    dt_y = send_and_wait(ser, f"set_y {cur_y}", SET_Y_ACK_RE)
                    if warmed_up and dt_x is not None and dt_y is not None:
                        send_s.append(dt_x + dt_y)
                        sent_this_iter = True
                elif args.send_mode == "fire_and_forget":
                    t_s0 = time.monotonic()
                    ser.write(f"set_x {cur_x}\n".encode("ascii"))
                    ser.write(f"set_y {cur_y}\n".encode("ascii"))
                    t_s1 = time.monotonic()
                    if warmed_up:
                        send_s.append(t_s1 - t_s0)
                        sent_this_iter = True
            t3 = time.monotonic()

            if warmed_up:
                n_iterations += 1
                capture_s.append(t1 - t0)
                detect_s.append(t2 - t1)
                if found is not None:
                    n_beam_found += 1
                if sent_this_iter:
                    n_sent += 1
                if t_prev_loop_start is not None:
                    total_s.append(t_loop_start - t_prev_loop_start)
            t_prev_loop_start = t_loop_start
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cam.stop()
        cam.close()
        if ser is not None:
            if args.send_mode == "fire_and_forget":
                try:
                    time.sleep(0.5)  # let the firmware drain before checking
                    _, dropped_after = get_cmdq_stats(ser)
                except Exception as e:
                    print(f"WARNING: couldn't read final cmdq_stats: {e}")
            try:
                ser.write(b"!")  # ISR-level hard stop, bypasses the line parser
            except Exception as e:
                print(f"WARNING: couldn't send emergency stop: {e}")
            ser.close()

    print(f"\n--send-mode={args.send_mode}  --raw-size={args.raw_size}  "
          f"{n_iterations} iterations over {args.duration_s}s (after {args.warmup_s}s warmup)")
    print(f"Beam detected: {n_beam_found}/{n_iterations} "
          f"({100.0 * n_beam_found / n_iterations:.1f}%)" if n_iterations else "No iterations completed.")
    if args.send_mode != "none":
        print(f"Sent both axes successfully: {n_sent}/{n_iterations}")

    report_ms("Achieved loop period (capture+detect+send, back-to-back)", total_s, n_iterations)
    report_ms("  capture stage", capture_s, n_iterations)
    report_ms("  detect stage", detect_s, n_iterations)
    if args.send_mode != "none":
        label = "  send stage (both axes, wait-for-ack)" if args.send_mode == "wait_ack" \
            else "  send stage (both axes, fire-and-forget write time)"
        report_ms(label, send_s, n_iterations)

    if dropped_before is not None and dropped_after is not None:
        dropped = dropped_after - dropped_before
        print(f"\ncmdq_stats dropped counter: +{dropped} over this run "
              f"({'OK, no drops' if dropped == 0 else 'SOME COMMANDS WERE DROPPED at this rate'})")

    out_path = args.out
    if out_path is None:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        out_path = f"results/fta_closed_loop_fps_{args.send_mode}_{args.raw_size}_{ts}.npz"
    np.savez(
        out_path,
        send_mode=args.send_mode, raw_size=args.raw_size,
        capture_s=np.array(capture_s), detect_s=np.array(detect_s),
        send_s=np.array(send_s), total_s=np.array(total_s),
        n_beam_found=n_beam_found, n_sent=n_sent, n_iterations=n_iterations,
    )
    print(f"Saved raw per-stage timing arrays to {out_path}")


if __name__ == "__main__":
    main()
