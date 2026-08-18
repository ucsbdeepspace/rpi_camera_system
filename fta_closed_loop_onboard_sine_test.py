#!/usr/bin/env python3
"""
Closed-loop sine tracking test using the FIRMWARE's own on-board sine
setpoint generator (added 2026-08-13, "emergency" alternative to
streaming target_x from the host) instead of streaming individual
set_target_x commands. One `start_sine FREQ_MILLIHZ AMPLITUDE_PX
CENTER_PX` command starts the Nucleo computing
target_x(t) = center + amplitude*sin(2*pi*freq*(t-t0)) itself, once per
control step, using its own HAL_GetTick() -- no host command stream
needed at all, sidestepping the VCP throughput ceiling
(fta_closed_loop_sine_response_test_vcp.py) documented in CLAUDE.md.

Measurement: the host already knows the exact commanded function (it
chose freq/amplitude/center), so it fits the MEASURED cx trace (from the
existing telemetry relay stream, unchanged) against the same known
sin(2*pi*freq*t) used for the open/closed-loop VCP-streamed sine tests --
no need for the firmware to report the realized target_x back over the
(bandwidth-limited) link.

Usage:
  python3 fta_closed_loop_onboard_sine_test.py --freq HZ
      [--amplitude-px N] [--base-dac-y N] [--kp-milli N] [--ki-milli N]
      [--duration SEC] [--port PORT] [--out PATH]
"""
import argparse
import math
import re
import threading
import time
from datetime import datetime, timezone

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FTA_BAUD = 460800
MICRONS_PER_PIXEL = 3.0

REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+pkts=(\d+)\s+errs=(\d+)$")
STATUS_FIELD_RE = {
    "dac_x": re.compile(r"dac_x=(-?\d+)"),
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_x": re.compile(r"tel_x=(-?[\d.]+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
}

BLUE = "#2a78d6"
ORANGE = "#eb6834"
TARGET_COLOR = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"


def find_fta_port():
    from serial.tools import list_ports
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    return candidates[0].device if candidates else None


def send_command(ser, cmd, char_delay=0.02, reply_timeout=2.0, retries=5):
    """Paced write (~20ms/char, this session's one proven-reliable rate
    for single commands), with retries -- we only ever send a handful of
    one-time setup commands here, so this doesn't need to be fast."""
    for attempt in range(retries):
        for ch in cmd + "\n":
            ser.write(ch.encode("ascii"))
            time.sleep(char_delay)
        deadline = time.monotonic() + reply_timeout
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if REPLY_RE.match(line):
                return line
    return None


def send_command_timed(ser, cmd, char_delay=0.02, reply_timeout=2.0, retries=5):
    """Like send_command, but also returns the host-side monotonic time
    right after the last character (the trailing '\\n') of the attempt
    that actually got a reply was written.

    Originally this precise timestamp mattered a lot -- it was used as
    the fit's t=0 reference, and the firmware parses/executes a command
    (latching g_sine_start_tick) the instant it finishes receiving the
    line, well before it starts transmitting a reply, so "when the OK
    reply arrived" baked in the command's full ~20ms/char transmit time
    (hundreds of ms for a ~25-char line) as a systematic "host t=0 is
    late" offset -- which read exactly like negative lag (the signal
    appearing to lead its own reference). That's no longer how lag gets
    computed (see fit_tracking's docstring -- it now diffs the fitted
    phase of the measured trace against the firmware's own per-sample
    reported tgt field, immune to any t0 error), so retrying here is
    safe now: a stale/duplicate start_sine landing doesn't corrupt the
    analysis the way it would have when t0 accuracy actually mattered.
    Kept timed (not just send_command) since t_sent is still used to
    seed the reader thread's relative clock, just no longer load-bearing
    for the reported gain/lag numbers."""
    for attempt in range(retries):
        for ch in cmd + "\n":
            ser.write(ch.encode("ascii"))
            time.sleep(char_delay)
        t_sent = time.monotonic()
        deadline = time.monotonic() + reply_timeout
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if REPLY_RE.match(line):
                return line, t_sent
    return None, t_sent


def get_status(ser, retries=5):
    for _ in range(retries):
        reply = send_command(ser, "get_status", retries=1)
        if reply is None or not reply.startswith("STATUS"):
            continue
        matches = {k: rx.search(reply) for k, rx in STATUS_FIELD_RE.items()}
        if all(matches.values()):
            return {
                "dac_x": int(matches["dac_x"].group(1)),
                "dac_y": int(matches["dac_y"].group(1)),
                "amp": int(matches["amp"].group(1)),
                "tel_x": float(matches["tel_x"].group(1)),
                "tel_age_ms": int(matches["tel_age_ms"].group(1)),
            }
    raise RuntimeError("No parseable get_status reply after several attempts.")


def fit_sine_component(t, y, w):
    basis = np.stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)], axis=1)
    coeffs, *_ = np.linalg.lstsq(basis, y, rcond=None)
    A, B, C = coeffs
    return float(A), float(B), float(C)


def fit_tracking(t, measured, target, freq):
    """Fits BOTH the measured cx trace and the firmware's own per-sample
    tgt trace against the same sin(wt)/cos(wt) basis (same t array), then
    takes the DIFFERENCE of their fitted phases as the lag.

    This is deliberately immune to any error in the host's t=0 reference
    (e.g. the send_command_timed estimate, or the even-worse "when the OK
    reply arrived" it replaced) -- a constant t0 offset shifts both
    fitted phases by the same amount, which cancels out of the
    difference. It also doesn't need to assume the commanded
    amplitude/center were exactly what was requested (the firmware's own
    sinf()/integer rounding could differ slightly) -- both are read from
    the fit of the real tgt trace instead. This replaces trusting an
    idealized sin(2*pi*freq*t) reference entirely."""
    w = 2.0 * math.pi * freq
    Ax, Bx, Cx = fit_sine_component(t, measured, w)
    At, Bt, Ct = fit_sine_component(t, target, w)
    amp_x = float(np.hypot(Ax, Bx))
    amp_t = float(np.hypot(At, Bt))
    phase_x = float(np.arctan2(Bx, Ax))
    phase_t = float(np.arctan2(Bt, At))
    gain = amp_x / amp_t if amp_t > 1e-6 else float("nan")
    # atan2 alone only guarantees each phase is within (-pi, pi], not
    # their difference -- wrap into (-pi, pi] (smallest-magnitude
    # equivalent) before converting to a lag, or a genuine ~90+ degree
    # lag can come out as e.g. -270 degrees ("leading" by 3/4 of a
    # period) instead of the equivalent, much more sensible +90 degrees.
    # This is still the fundamental single-frequency wraparound ambiguity
    # (can't distinguish lag from lag +/- n*period) -- just resolved to
    # its smallest-magnitude branch rather than left to alias arbitrarily
    # far past +/-180 degrees.
    phase_diff = (phase_x - phase_t + math.pi) % (2.0 * math.pi) - math.pi
    lag_ms = -phase_diff / w * 1000.0
    return gain, lag_ms, Cx - Ct


def save_plot(t, x, tgt, dac_y, freq, amplitude_px, base_dac_y, kp_milli, ki_milli, gain, lag_ms, out_path):
    """Primary axis is um (the physically meaningful unit for this
    project's actual deliverable -- beacon-wobble rejection in real
    displacement), with px kept as a secondary axis rather than dropped
    entirely, since every DAC-side reasoning elsewhere in this project
    still happens in px/counts. Second panel is the real commanded
    actuator output (dac_y, raw DAC counts) over the same time axis --
    added 2026-08-14 so the actuator command is visible directly
    alongside the resulting cx/tgt trace, instead of only being
    inferable offline from the control law."""
    um = MICRONS_PER_PIXEL
    period_ms = 1000.0 / freq
    lag_deg = (lag_ms / period_ms) * 360.0

    fig, (ax, ax_dac) = plt.subplots(2, 1, figsize=(10, 6.5), dpi=150, sharex=True,
                                       gridspec_kw={"height_ratios": [1.6, 1], "hspace": 0.12})
    for a in (ax, ax_dac):
        a.set_facecolor("white")
        for spine in ("top", "right"):
            a.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            a.spines[spine].set_color(GRID)
        a.tick_params(colors=MUTED, labelsize=9, length=3)

    ax.plot(t, tgt * um, color=TARGET_COLOR, linewidth=1.1, linestyle=(0, (2, 2)),
            label="target_x (firmware-reported, per-sample)")
    ax.plot(t, x * um, color=BLUE, linewidth=1.3, label="measured cx")

    sec = ax.secondary_yaxis("right", functions=(lambda v: v / um, lambda px: px * um))
    sec.tick_params(colors=MUTED, labelsize=9, length=3)
    sec.set_ylabel("px", fontsize=9, color=MUTED)

    ax.set_ylabel("cx (µm)", fontsize=9.5, color=MUTED)
    ax.legend(fontsize=9, loc="upper right", facecolor="white", edgecolor=GRID, framealpha=0.9)

    parts = [f"{freq}Hz  amplitude={amplitude_px:.2f}px / {amplitude_px*um:.1f}um "
             f"@ dac_y={base_dac_y} (on-board sine gen)",
             f"Kp={kp_milli/1000:.2f} Ki={ki_milli/1000:.2f}",
             f"gain: {gain:.2f}  lag: {lag_ms:.1f}ms ({lag_deg:.0f}°)"]
    ax.text(0.02, 0.03, "\n".join(parts), transform=ax.transAxes, fontsize=8.5,
            color="#0b0b0b", va="bottom", ha="left",
            bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=4))

    ax_dac.plot(t, dac_y, color=ORANGE, linewidth=1.1)
    ax_dac.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
    ax_dac.set_ylabel("dac_y (counts)", fontsize=9.5, color=MUTED)

    fig.suptitle(f"Closed-loop sine tracking (on-board generator), {freq}Hz", fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, facecolor="white")
    plt.close(fig)


def _reader_thread(ser, t0, records, stop_event):
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
            continue
        status = int(m.group(2))
        if not (status & 1):
            continue
        x = float(m.group(3))
        tgt = float(m.group(5))
        dac_y = int(m.group(6))
        records.append((now, x, tgt, dac_y))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--freq", type=float, default=None,
                         help="required unless --replot is given")
    parser.add_argument("--amplitude-px", type=float, default=25.0)
    parser.add_argument("--base-dac-y", type=int, default=2048)
    parser.add_argument("--kp-milli", type=int, default=1750)
    parser.add_argument("--ki-milli", type=int, default=200000)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    parser.add_argument("--replot", default=None,
                         help="skip hardware entirely -- reload an existing results/*.npz and "
                              "just regenerate its PNG (e.g. after a plotting/unit change)")
    args = parser.parse_args()

    if args.replot:
        d = np.load(args.replot)
        out_path = args.out or args.replot.rsplit(".", 1)[0] + ".png"
        # dac_y wasn't recorded before 2026-08-14 -- older npz files won't
        # have it; fall back to NaN (renders as a gap, not a misleading flat
        # line) rather than erroring out on a re-plot of older data.
        dac_y = d["dac_y"] if "dac_y" in d.files else np.full_like(d["t"], np.nan)
        save_plot(d["t"], d["x"], d["tgt"], dac_y, float(d["freq"]), float(d["amplitude_px"]),
                  int(d["base_dac_y"]), int(d["kp_milli"]), int(d["ki_milli"]),
                  float(d["gain"]), float(d["lag_ms"]), out_path)
        print(f"Replotted {args.replot} -> {out_path}")
        return

    if args.freq is None:
        parser.error("--freq is required unless --replot is given")

    duration = args.duration if args.duration is not None else max(2.0, 8.0 / args.freq)

    import serial

    port = args.port or find_fta_port()
    if port is None:
        print("No ST-Link serial port found.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    print(send_command(ser, "clear_estop"))
    print(send_command(ser, "set_mode open_loop"))

    st = get_status(ser)
    if st["tel_age_ms"] > 500:
        print(f"ERR: last relayed telemetry is {st['tel_age_ms']}ms old -- nothing streaming from the Pi.")
        ser.close()
        raise SystemExit(1)

    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        print(send_command(ser, "amp_enable"))
        st = get_status(ser)
        if not st["amp"]:
            print("ERR: amp_enable didn't take -- aborting.")
            ser.close()
            raise SystemExit(1)

    print(f"Pre-positioning dac_y={args.base_dac_y}...")
    print(send_command(ser, f"set_y {args.base_dac_y}"))
    time.sleep(0.5)

    st = get_status(ser)
    center_px = st["tel_x"]
    print(f"baseline cx={center_px:.1f}  amplitude={args.amplitude_px}px  freq={args.freq}Hz  "
          f"duration={duration:.2f}s  Kp={args.kp_milli/1000:.2f} Ki={args.ki_milli/1000:.2f}")

    print(send_command(ser, f"set_target_x {round(center_px)}"))
    print(send_command(ser, f"set_kp {args.kp_milli}"))
    print(send_command(ser, f"set_ki {args.ki_milli}"))
    print(send_command(ser, "set_mode closed_loop"))
    time.sleep(0.3)

    freq_millihz = round(args.freq * 1000)
    amplitude_x10 = round(args.amplitude_px * 10)
    start_reply, t_sine_start = send_command_timed(
        ser, f"start_sine {freq_millihz} {amplitude_x10} {round(center_px)}")
    print(start_reply)
    if not start_reply or not start_reply.startswith("OK"):
        print("ERR: start_sine failed -- aborting.")
        send_command(ser, "set_mode open_loop")
        send_command(ser, "set_y 95")
        if not amp_was_enabled:
            send_command(ser, "amp_disable")
        ser.close()
        raise SystemExit(1)
    # t_sine_start is the moment the last character ('\n') was written --
    # a much closer estimate of the firmware's true g_sine_start_tick
    # moment than waiting for the OK reply to arrive (see
    # send_command_timed's docstring).

    records = []
    stop_event = threading.Event()
    t0 = t_sine_start
    reader = threading.Thread(target=_reader_thread, args=(ser, t0, records, stop_event), daemon=True)
    ser.reset_input_buffer()
    reader.start()

    time.sleep(duration)

    stop_event.set()
    reader.join(timeout=1.0)

    print(send_command(ser, "stop_sine"))
    print(send_command(ser, "set_mode open_loop"))
    print(send_command(ser, "set_y 95"))
    if not amp_was_enabled:
        print(send_command(ser, "amp_disable"))
    print("final:", get_status(ser))
    ser.close()

    print(f"Captured {len(records)} telemetry samples over {duration:.2f}s "
          f"(~{len(records)/duration:.0f}/s average).")
    if len(records) < 6:
        print("Not enough samples to analyze.")
        return

    t = np.array([r[0] for r in records])
    x = np.array([r[1] for r in records])
    tgt = np.array([r[2] for r in records])
    dac_y = np.array([r[3] for r in records])

    gain, lag_ms, offset = fit_tracking(t, x, tgt, args.freq)
    period_ms = 1000.0 / args.freq
    lag_deg = (lag_ms / period_ms) * 360.0
    um = MICRONS_PER_PIXEL
    print(f"\ntracking gain: {gain:.3f} ({gain*100:.1f}% of commanded {args.amplitude_px}px amplitude, "
          f"{gain*args.amplitude_px:.1f}px / {gain*args.amplitude_px*um:.1f}um)")
    print(f"lag: {lag_ms:.1f}ms ({lag_deg:.1f} deg at {args.freq}Hz)  "
          f"[measured against the firmware's own per-sample tgt field, not a reconstructed reference]")
    print(f"offset from center: {offset:.2f}px ({offset*um:.1f}um)")
    print(f"implied |S|=|1-T| ~= {abs(1-gain):.3f} (magnitude-only approximation)")

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_path = args.out or f"results/fta_closed_loop_onboard_sine_{args.freq:g}Hz_{ts}.npz"
    np.savez(out_path, t=t, x=x, tgt=tgt, dac_y=dac_y, freq=args.freq, amplitude_px=args.amplitude_px,
              center_px=center_px, base_dac_y=args.base_dac_y, kp_milli=args.kp_milli, ki_milli=args.ki_milli,
              gain=gain, lag_ms=lag_ms, offset=offset)
    print(f"Saved raw time series to {out_path}")

    png_path = out_path.rsplit(".", 1)[0] + ".png"
    save_plot(t, x, tgt, dac_y, args.freq, args.amplitude_px, args.base_dac_y,
              args.kp_milli, args.ki_milli, gain, lag_ms, png_path)
    print(f"Saved plot to {png_path}")


if __name__ == "__main__":
    main()
