#!/usr/bin/env python3
"""
Commands a sine wave on one DAC axis and measures how well the actual pixel
position tracks it -- the frequency-domain complement to
fta_step_response_test_vcp.py's transient (step) characterization. Directly
relevant to the project's actual goal (top of CLAUDE.md): rejecting a
10-20Hz beacon-wobble disturbance. A step response tells you settling time
for a one-off jump; this tells you the actuator's steady-state gain and
phase lag AT a disturbance-band frequency, which is what a PID controller
actually has to reject continuously.

Same architecture as fta_step_response_test_vcp.py: runs entirely from the
laptop over the Nucleo's VCP, no Pi access needed as long as something
there (camera_view_tool.py or beam_position_streamer.py) is already
streaming telemetry. A background reader thread drains the relayed
per-packet telemetry lines (~170-190/s, confirmed in bench testing)
into timestamped (t, x, y) samples while the main thread paces
`set_x`/`set_y` commands at --update-rate to trace out
center + amplitude*sin(2*pi*freq*t).

**Real ceiling not yet characterized**: how fast this firmware's ASCII
line-based VCP command parsing can reliably accept `set_x`/`set_y` without
dropping updates hasn't been measured (the old "FTA Controller" firmware's
1600Hz fire-and-forget ceiling doesn't carry over -- different protocol
entirely). Start with a low --freq / modest --update-rate to sanity-check
the commanded trace still looks sinusoidal (not aliased/stepped) before
pushing toward the actual 10-20Hz disturbance band.

Analysis: since the test frequency is known exactly (we generate the
command), fitting the measured pixel trace to
A*sin(2*pi*f*t) + B*cos(2*pi*f*t) + C is a *linear* least-squares problem
(no iterative fitting needed) -- solved for both the driven axis and the
other axis (to see cross-axis coupling, same idea as the step test's
dominant-vs-secondary axis). Reports fitted amplitude (px), phase lag
(ms, relative to the commanded sin(2*pi*f*t) with zero phase by
construction), and offset, for both axes.

Usage:
  python3 fta_sine_response_test_vcp.py --axis x|y --freq HZ
      [--amplitude N] [--center N] [--duration SEC] [--update-rate HZ]
      [--port PORT] [--out PATH]

    --axis          which DAC channel to drive, default x
    --freq          sine frequency in Hz -- required, no default (this is
                    the whole point of the test, a deliberate choice)
    --amplitude N   DAC counts, peak deviation from --center, default 200
                    (same small-step regime already characterized as
                    well-behaved in the step-response tests)
    --center N      DAC counts, sine midpoint, default 2000
    --duration SEC  how long to run the sine, default enough for 8 full
                    cycles at --freq (min 2.0s)
    --update-rate HZ  how often to send a new set_x/set_y value, default
                    200. Should be well above --freq (>=10x) for the
                    commanded trajectory to look sinusoidal rather than a
                    coarse staircase.
    --port PORT     Nucleo VCP serial port, default auto-detect
    --out PATH      where to save the raw (t, x, y) time series plus the
                    exact commanded trajectory -- default
                    results/fta_sine_response_vcp_<axis>_<freq>Hz_<UTC
                    timestamp>.npz

Requires the Pi to already be streaming telemetry -- checks freshness
before starting and fails fast with a clear message if nothing is
streaming.
"""
import argparse
import math
import re
import threading
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
    few times -- the VCP occasionally drops a character out of a command
    under live I2C load (much rarer since the 2026-08-04 NVIC priority
    fix, but not impossible); the firmware always replies ERR to a
    corrupted line rather than hanging, so a fresh attempt recovers."""
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


def _reader_thread(ser, t0, records, stop_event):
    """Continuously drains BOTH telemetry lines and this script's own
    set_x/set_y OK/ERR replies -- during the sine command loop we don't
    read replies synchronously (that would break the pacing), so this
    thread has to keep the serial receive buffer from backing up, or the
    firmware's blocking HAL_UART_Transmit for each reply could eventually
    stall waiting for host-side buffer space."""
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
            continue  # OK/ERR reply or heartbeat -- drained, not needed here
        status = int(m.group(2))
        if not (status & 1):
            continue  # beam not confidently detected this cycle
        x = float(m.group(3))
        y = float(m.group(4))
        records.append((now, x, y))


def fit_sine(t, v, freq, expected_sign=None):
    """Linear least-squares fit of v ~= A*sin(wt) + B*cos(wt) + C, w known
    exactly (we commanded it) so this is linear, not an iterative fit.

    Returns (signed_gain, lag_rad, offset), NOT the raw (amplitude, phase)
    a naive read of the fit would give. sqrt(A^2+B^2) is always positive
    and folds the actual sign of the DAC->pixel relationship into phase --
    for an axis where increasing DAC moves the pixel value DOWN (confirmed
    directly from the step-response data: DAC 95->2000 on x measured a
    -148.6px delta), phase lands near +-180 degrees at EVERY frequency
    regardless of any real dynamics, since a sign flip IS a constant
    180-degree phase shift by definition. Mistaking that for lag was a
    real error made analyzing the first pass of this data (2026-08-06) --
    it produced a "large lag at every frequency including near-DC" result
    that isn't physically sensible for a system whose step-response
    settling times are under a second.

    Without expected_sign, the reference (0 or 180 degrees) is auto-
    detected by choosing whichever puts the residual phase within +-90
    degrees. That auto-detection is only reliable while true lag stays
    under 90 degrees -- confirmed to break at 5Hz (2026-08-06): axis x's
    known-negative gain was misdetected as positive, with an impossible
    negative "lag" as the symptom, once real actuator dynamics stacked on
    top of the ~41ms pipeline delay pushed total phase past the boundary.
    Since a linear system's static-gain SIGN is a fixed physical property
    that cannot change with test frequency (confirmed independently by
    the step-response data and every 0.1-2Hz sine run here), pass
    expected_sign (+1.0 or -1.0) to fix it instead of re-guessing it per
    frequency -- valid up to a full +-180 degrees of real lag before the
    fundamental single-frequency wrap ambiguity reappears, a much wider
    safe margin than +-90."""
    w = 2.0 * math.pi * freq
    basis = np.stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)], axis=1)
    coeffs, *_ = np.linalg.lstsq(basis, v, rcond=None)
    A, B, C = coeffs
    amplitude = float(np.hypot(A, B))
    phase = float(np.arctan2(B, A))  # in (-pi, pi]

    if expected_sign is not None:
        sign = 1.0 if expected_sign > 0 else -1.0
        if sign > 0:
            lag_rad = phase
        else:
            lag_rad = phase - math.pi if phase > 0 else phase + math.pi
    elif abs(phase) <= math.pi / 2:
        sign = 1.0
        lag_rad = phase
    else:
        sign = -1.0
        lag_rad = phase - math.pi if phase > 0 else phase + math.pi

    return sign * amplitude, lag_rad, float(C)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--axis", choices=["x", "y"], default="x")
    parser.add_argument("--freq", type=float, required=True)
    parser.add_argument("--amplitude", type=int, default=200)
    parser.add_argument("--center", type=int, default=2000)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--update-rate", type=float, default=200.0)
    parser.add_argument("--gain-sign", type=float, choices=[-1.0, 1.0], default=-1.0,
                         help="Fixed sign for the DAC->pixel gain (see fit_sine's "
                              "docstring) -- default -1.0 matches every axis-x "
                              "measurement so far (step response + 0.1-2Hz sine).")
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    duration = args.duration if args.duration is not None else max(2.0, 8.0 / args.freq)
    if args.update_rate < 10 * args.freq:
        print(f"WARNING: --update-rate ({args.update_rate}) is less than 10x --freq "
              f"({args.freq}) -- the commanded trajectory will look like a coarse "
              "staircase, not a smooth sine.")

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
    print(f"Driving axis '{args.axis}': center={args.center} amplitude={args.amplitude} "
          f"freq={args.freq}Hz duration={duration:.2f}s update_rate={args.update_rate}Hz "
          f"({int(duration * args.update_rate)} commands).")

    amp_was_enabled = bool(amp)
    if not amp_was_enabled:
        print("Amp is currently disabled -- enabling it for this test "
              "(will be restored to disabled afterward).")
        ser.write(b"amp_enable\n")
        time.sleep(0.1)
        # Verify, don't assume -- a fire-and-forget enable here previously
        # produced a real false result (2026-08-06 0.1Hz run: amp was
        # actually still off, measured as ~zero response, initially
        # mistaken for a real high-pass/AC-coupling finding about the
        # actuator). Confirm the state actually changed before trusting
        # any data collected after this point.
        _, _, amp_confirmed, _ = get_status(ser)
        if not amp_confirmed:
            print("ERR: sent amp_enable but get_status still reports amp=off -- "
                  "aborting rather than collecting data with the amp off again.")
            ser.close()
            raise SystemExit(1)

    records = []
    commanded = []  # (t, value) at the exact moment each command was sent
    stop_event = threading.Event()
    try:
        # Settle at center before recording/driving anything.
        ser.write(f"set_{args.axis} {args.center}\n".encode("ascii"))
        time.sleep(0.5)

        # Telemetry keeps arriving during the settle sleep above with nobody
        # draining it -- without this, the reader thread's first read would
        # burst through that whole backlog in a few ms, stamping samples
        # spanning up to the last ~0.5s with nearly the same timestamp. That
        # would corrupt a sine-phase fit (unlike the step-response script,
        # which only ever takes a mean over its pre-step baseline window --
        # insensitive to *relative* timing within that window, so it doesn't
        # need this fix).
        ser.reset_input_buffer()

        t0 = time.monotonic()
        reader = threading.Thread(target=_reader_thread, args=(ser, t0, records, stop_event), daemon=True)
        reader.start()

        dt_cmd = 1.0 / args.update_rate
        n_samples = int(duration * args.update_rate)
        w = 2.0 * math.pi * args.freq
        for i in range(n_samples):
            t_cmd = i * dt_cmd
            value = int(round(args.center + args.amplitude * math.sin(w * t_cmd)))
            ser.write(f"set_{args.axis} {value}\n".encode("ascii"))
            commanded.append((t_cmd, value))
            target = t0 + t_cmd + dt_cmd
            sleep_s = target - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

        actual_span = time.monotonic() - t0
        time.sleep(0.3)  # let the last few telemetry samples land
        stop_event.set()
        reader.join(timeout=1.0)
    finally:
        ser.write(f"set_{args.axis} {args.center}\n".encode("ascii"))
        time.sleep(0.1)
        if not amp_was_enabled:
            ser.write(b"amp_disable\n")
            time.sleep(0.1)
        ser.close()

    print(f"Command loop took {actual_span:.3f}s for {n_samples} commands "
          f"(target {duration:.3f}s) -- achieved update rate "
          f"~{n_samples / actual_span:.0f}Hz.")

    if len(records) < 3 * args.freq * duration:
        print(f"Only {len(records)} usable telemetry samples over {duration:.2f}s -- "
              "may not be enough to fit a clean sine. Check the beam is visible "
              "throughout, or that --update-rate isn't overwhelming the link.")

    t = np.array([r[0] for r in records])
    x = np.array([r[1] for r in records])
    y = np.array([r[2] for r in records])
    cmd_t = np.array([c[0] for c in commanded])
    cmd_v = np.array([c[1] for c in commanded], dtype=float)

    driven, other = (x, y) if args.axis == "x" else (y, x)
    driven_name, other_name = (args.axis, "y" if args.axis == "x" else "x")

    # signed_gain's sign reflects the DAC->pixel relationship's real
    # direction (e.g. negative = DAC up moves the pixel value down); the
    # residual lag_rad is the actual, small, frequency-dependent delay --
    # see fit_sine's docstring for why this isn't just "amplitude, phase".
    # expected_sign is fixed (not auto-detected) from prior evidence: both
    # the step-response data and every 0.1-2Hz sine run on this axis found
    # a negative self-gain (DAC up -> pixel down) and negative cross-axis
    # gain -- args.gain_sign defaults to that, override with --gain-sign 1
    # if ever driving/checking an axis where that doesn't hold.
    gain_driven, lag_rad_driven, off_driven = fit_sine(t, driven, args.freq, args.gain_sign)
    gain_other, lag_rad_other, off_other = fit_sine(t, other, args.freq, args.gain_sign)

    lag_ms_driven = -lag_rad_driven / w * 1000.0
    lag_ms_other = -lag_rad_other / w * 1000.0

    print(f"\n--- Driven axis ({driven_name}) ---")
    print(f"fitted gain: {gain_driven:+.2f}px for {args.amplitude:+d} commanded DAC counts "
          f"({'DAC up -> pixel up' if gain_driven > 0 else 'DAC up -> pixel down'})")
    print(f"lag: {lag_ms_driven:.1f}ms  ({lag_rad_driven * 180/math.pi:.1f} deg at {args.freq}Hz, "
          "relative to the fitted gain direction, not raw phase)")
    print(f"offset: {off_driven:.2f}px")

    print(f"\n--- Cross-coupled axis ({other_name}) ---")
    print(f"fitted gain: {gain_other:+.2f}px  ({100*abs(gain_other)/max(abs(gain_driven),1e-9):.1f}% "
          f"of driven-axis magnitude, {'same' if gain_other*gain_driven > 0 else 'opposite'} sign)")
    print(f"lag: {lag_ms_other:.1f}ms")
    print(f"offset: {off_other:.2f}px")

    out_path = args.out
    if out_path is None:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        out_path = f"results/fta_sine_response_vcp_{args.axis}_{args.freq:g}Hz_{ts}.npz"
    np.savez(out_path, t=t, x=x, y=y, cmd_t=cmd_t, cmd_v=cmd_v,
             axis=args.axis, freq=args.freq, amplitude=args.amplitude, center=args.center,
             fit_driven=(gain_driven, lag_rad_driven, off_driven),
             fit_other=(gain_other, lag_rad_other, off_other))
    print(f"\nSaved raw time series to {out_path}")


if __name__ == "__main__":
    main()
