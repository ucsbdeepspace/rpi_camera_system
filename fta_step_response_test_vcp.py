#!/usr/bin/env python3
"""
Characterizes the FTA's actual DYNAMIC response (rise time, settling time,
overshoot) to a step change in DAC setpoint -- same measurement and analysis
as fta_step_response_test.py, but sourced entirely differently.

fta_step_response_test.py captures camera frames directly via Picamera2, so
it can only run ON the Pi. Under the current architecture (CLAUDE.md,
"Architecture DECISION v2", 2026-08-04) the Pi's only job is capturing
frames and streaming centroids to the Nucleo over I2C -- it doesn't need to
run this test itself. The Nucleo already prints a line over its VCP for
EVERY relayed I2C packet (see camera_centroid_receiver's main.c,
g_new_packet_ready handling) at the same rate the Pi streams -- typically
hundreds of Hz, confirmed live during 2026-08-04 bench testing. That's a
ready-made high-rate position feed over the exact same serial link this
script already uses to command set_x/set_y, so THIS version runs entirely
from the laptop: no Picamera2/cv2 dependency, no SSH/access to the Pi
needed, as long as something on the Pi (camera_view_tool.py or
beam_position_streamer.py) is already running and streaming.

Tradeoff vs. the camera-direct version: time resolution here is bounded by
the I2C relay rate and VCP line framing/read-thread overhead, not raw
camera fps, and there's a small added (roughly constant) relay latency
between actual camera capture and this script seeing the line -- baseline/
step timing is still measured relative to when THIS script sent the step
command, so that latency mostly cancels out, but it isn't zero the way
frame-timestamp-at-capture was. If sub-frame timing precision is ever
needed again, fta_step_response_test.py (run on the Pi) is the ground-truth
version; this one is for exactly the case discussed 2026-08-04 -- rerunning
the existing small-step characterization after the SB16/SB18 hardware fix,
without needing Pi access.

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

Both x and y are logged regardless of which axis was stepped -- this also
surfaces cross-axis coupling, same as the camera-direct version.

Usage:
  python3 fta_step_response_test_vcp.py --step-to N [--axis x|y]
      [--step-from N] [--pre-s SEC] [--post-s SEC] [--settle-tol-px PX]
      [--port PORT] [--out PATH]

    --axis            which DAC channel to step, default x
    --step-from N      starting DAC setpoint, commanded and allowed to
                       settle before recording begins. Default: read the
                       Nucleo's own last commanded position via get_status.
    --step-to N        DAC setpoint to step TO -- required, no default,
                       a step size is a deliberate per-setup choice.
    --pre-s SEC        seconds of baseline recorded BEFORE the step,
                       default 0.2
    --post-s SEC       seconds recorded AFTER the step, default 1.0 --
                       widen this if the actuator might settle slower
                       than that.
    --settle-tol-px PX pixel tolerance band for "settled", default 2.0
                       (= 6.0um at the OV9281's 3.0um/px pitch, same
                       constant fta_step_response_test.py uses).
    --port PORT        Nucleo VCP serial port, default auto-detect.
    --out PATH         where to save the raw (t, x, y) time series --
                       default results/fta_step_response_vcp_<axis>_<UTC
                       timestamp>.npz

Requires the Pi to already be streaming telemetry (camera_view_tool.py or
beam_position_streamer.py running there) -- this script checks telemetry
freshness before starting and fails fast with a clear message if nothing
is streaming, rather than silently recording zero usable samples.
"""
import argparse
import re
import threading
import time
from datetime import datetime, timezone

import numpy as np

# camera_centroid_receiver's phase-1 firmware (nucleo_firmware/, 2026-08-04)
# USART2 rate -- NOT the old "FTA Controller"'s 460800.
FTA_BAUD = 115200

MICRONS_PER_PIXEL = 3.0  # OV9281 pixel pitch, same constant as fta_step_response_test.py

# Per-relayed-packet line printed by main.c's main loop, e.g.:
#   "seq= 63 status=1 x=965.6 y=563.5 pkts=3521 errs=0"
# x/y are already POSITION_SCALE-descaled real pixel values (one decimal
# digit), not raw wire units -- no further scaling needed here.
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+pkts=(\d+)\s+errs=(\d+)$")

# get_status reply, e.g.:
#   "STATUS mode=open_loop amp=1 estop=0 dac_x=200 dac_y=200 tel_x=965.6
#    tel_y=567.2 tel_seq=145 tel_status=1 tel_age_ms=0 pkts=7187 errs=0 uptime=36s"
# Field order in the firmware's snprintf is mode, amp, estop, dac_x, dac_y,
# ... (amp comes BEFORE dac_x) -- match each field independently rather
# than assuming a sequential order in one regex.
STATUS_FIELD_RE = {
    "dac_x": re.compile(r"dac_x=(-?\d+)"),
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
}


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


def get_status(ser):
    """Returns (dac_x, dac_y, amp, tel_age_ms). Retries the whole request a
    few times -- bench testing this firmware (2026-08-04) found the VCP
    occasionally drops a character out of a command when it lands during
    the Pi's high-rate I2C telemetry stream, corrupting e.g. "get_status"
    into "ge_status". The firmware always replies ERR to a corrupted line
    rather than hanging, so a fresh attempt is enough to recover."""
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


def analyze_step(t, primary, t_step, settle_tol_px):
    """Identical to fta_step_response_test.py's analyze_step -- duplicated
    per this project's established convention of not sharing helpers across
    these one-off test scripts (see e.g. find_beam_blob's docstring in that
    file). t, primary are numpy arrays of (timestamp, pixel coordinate) for
    the stepped axis's own pixel trace."""
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


def _reader_thread(ser, t0, records, stop_event):
    """Runs on its own thread so the main thread's step-command timing
    (time.sleep(args.pre_s), write, time.sleep(args.post_s)) isn't coupled
    to serial read latency -- concurrent read (here) + write (main thread)
    on the same pyserial Serial object is a standard, safe pattern."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except Exception:
            continue
        if not raw:
            continue
        now = time.monotonic() - t0
        m = TELEMETRY_RE.match(raw.decode(errors="replace").strip())
        if not m:
            continue  # heartbeat line, a command's OK/ERR reply, or a corrupted line
        status = int(m.group(2))
        if not (status & 1):
            continue  # beam not confidently detected this cycle
        x = float(m.group(3))
        y = float(m.group(4))
        records.append((now, x, y))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--step-to", type=int, required=True)
    parser.add_argument("--axis", choices=["x", "y"], default="x")
    parser.add_argument("--step-from", type=int, default=None)
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
              "check the Nucleo's USB cable is connected to this machine.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    dac_x, dac_y, amp, tel_age_ms = get_status(ser)
    if tel_age_ms > 500:
        print(f"ERR: last relayed I2C telemetry is {tel_age_ms}ms old -- nothing "
              "appears to be streaming from the Pi. Start camera_view_tool.py or "
              "beam_position_streamer.py there first, then retry.")
        raise SystemExit(1)
    print(f"Telemetry is live (last packet {tel_age_ms}ms old). "
          f"Current DAC position: x={dac_x} y={dac_y}, amp={'on' if amp else 'off'}.")

    step_from = args.step_from if args.step_from is not None else (dac_x if args.axis == "x" else dac_y)
    print(f"Stepping axis '{args.axis}' from {step_from} to {args.step_to}.")

    amp_was_enabled = bool(amp)
    if not amp_was_enabled:
        print("Amp is currently disabled -- enabling it for this test "
              "(will be restored to disabled afterward).")
        ser.write(b"amp_enable\n")
        time.sleep(0.1)

    records = []
    stop_event = threading.Event()
    try:
        # Settle at step_from before recording anything.
        ser.write(f"set_{args.axis} {step_from}\n".encode("ascii"))
        time.sleep(0.5)

        t0 = time.monotonic()
        reader = threading.Thread(target=_reader_thread, args=(ser, t0, records, stop_event), daemon=True)
        reader.start()

        time.sleep(args.pre_s)
        ser.write(f"set_{args.axis} {args.step_to}\n".encode("ascii"))
        t_step = time.monotonic() - t0
        time.sleep(args.post_s)

        stop_event.set()
        reader.join(timeout=1.0)
    finally:
        # Leave the board in a clean idle state, matching the 2026-08-04
        # bench-test convention -- back to step_from, amp restored to
        # whatever it was before this script touched it.
        ser.write(f"set_{args.axis} {step_from}\n".encode("ascii"))
        time.sleep(0.1)
        if not amp_was_enabled:
            ser.write(b"amp_disable\n")
            time.sleep(0.1)
        ser.close()

    if len(records) < 6:
        print(f"Only {len(records)} usable telemetry samples -- not enough to analyze. "
              "Check the beam is actually visible/detected on the Pi throughout.")
        return

    t = np.array([r[0] for r in records])
    x = np.array([r[1] for r in records])
    y = np.array([r[2] for r in records])

    # Don't assume DAC-x drives pixel-x -- auto-pick whichever pixel axis
    # actually moved more between the pre- and post-step windows, same
    # logic as fta_step_response_test.py.
    pre_mask, post_mask = t < t_step, t >= t_step
    delta_x = abs(x[post_mask].mean() - x[pre_mask].mean()) if pre_mask.any() and post_mask.any() else 0.0
    delta_y = abs(y[post_mask].mean() - y[pre_mask].mean()) if pre_mask.any() and post_mask.any() else 0.0
    if delta_y > delta_x:
        primary, primary_name = y, "y"
    else:
        primary, primary_name = x, "x"
    print(f"Dominant response axis: {primary_name} (|delta x|={delta_x:.1f}px, "
          f"|delta y|={delta_y:.1f}px) -- analyzing {primary_name}.")

    span = t[-1] - t[0]
    print(f"Captured {len(records)} telemetry samples ({span:.3f}s span, "
          f"~{len(records) / span if span > 0 else 0:.0f}/s average).")

    metrics = analyze_step(t, primary, t_step, args.settle_tol_px)
    if metrics is None:
        print("Not enough samples before/after the step to compute metrics.")
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
        out_path = f"results/fta_step_response_vcp_{args.axis}_{ts}.npz"
    np.savez(out_path, t=t, x=x, y=y, t_step=t_step,
             step_from=step_from, step_to=args.step_to, axis=args.axis)
    print(f"Saved raw time series to {out_path}")


if __name__ == "__main__":
    main()
