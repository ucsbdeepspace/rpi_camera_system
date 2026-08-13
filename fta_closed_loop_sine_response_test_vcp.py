#!/usr/bin/env python3
"""
Drives the CLOSED-LOOP dac_y->cx PID pathway's target_x setpoint through a
sinusoid and measures how well the measured cx tracks it -- the frequency-
domain complement to fta_closed_loop_step_response_vcp.py's transient (step)
characterization, and the closed-loop analog of the open-loop
fta_sine_response_test_vcp.py.

Why tracking a moving SETPOINT measures disturbance REJECTION: for a
unity-feedback loop, the reference-tracking transfer function T(s) and the
disturbance-rejection sensitivity S(s) are complementary, S(s) + T(s) = 1.
Good tracking of a moving target_x (T near 1, i.e. measured cx closely
follows commanded target_x in both gain and phase) at some frequency
directly implies good rejection of a real disturbance at that same
frequency (S near 0). This is the actual project deliverable (top of
CLAUDE.md): confirming the loop can reject a 10-20Hz beacon wobble.

Unlike the open-loop sine script (which drives raw DAC counts and can have
either sign of DAC->pixel gain, requiring the sign-handling machinery in
its fit_sine), here both the driven signal (target_x, pixels) and the
measured signal (cx, pixels) are in the same units and the same sign by
construction -- a working closed loop's tracking gain should always be
positive (ideally near 1.0), so no sign ambiguity handling is needed.

Command reliability -- a REAL, hard throughput ceiling found the hard way
(2026-08-13), not a design choice: a first version fired fire-and-forget
BURST writes at --update-rate like the open-loop script's set_x/set_y
loop. Result: 100% of 1600 attempts at 200Hz were silently dropped --
target_x never moved from its primed value ONCE, all 8s of "sine"
recorded was just noise around the resting position. Switching to a
paced per-character write (like the setup commands) fixed correctness
but immediately hit a SECOND, confounding bug: Windows' default ~15.6ms
timer tick was silently inflating every `time.sleep(0.001)` call to
~15.6ms regardless of the requested value (measured directly: 50x
sleep(0.001) took 0.78s, not ~0.05s) -- meaning an earlier back-to-back
reliability probe that seemed to show 1ms/char was as safe as 20ms/char
was comparing two settings that were secretly running at the SAME real
delay the whole time. Fixed the confound with `winmm.timeBeginPeriod(1)`
(see main(), a standard Windows high-resolution-timer request) -- with
that in place, true ~1-4ms/char pacing measurably exists, and turned out
to be genuinely unreliable in this script's real multi-threaded context
(a concurrent reader thread appears to introduce scheduling jitter an
isolated single-threaded probe doesn't show -- 1.5ms/char tested 100%
reliable in isolation but only 0.2% applied here). **The only pacing
confirmed reliable in this actual multi-threaded sine-loop context is
the original ~20ms/char** used for one-shot setup commands elsewhere in
this project's 2026-08-13 tooling -- which caps real achievable target_x
updates at only ~3Hz (a ~17-character "set_target_x N\n" line takes
~340ms to send paced). **This is nowhere near enough to trace a valid
sine even at 1Hz** (10x-oversampling would need ~10Hz, 30x short of what
1Hz needs and ~100-200x short of the actual 10-20Hz disturbance band this
project targets) -- confirmed empirically: a 1Hz/3Hz-update-rate run
produced a coarse triangle wave, not a sine, and the real closed-loop
response visibly saturates against it rather than tracking a clean
sinusoid. **This script, as committed, cannot yet properly characterize
closed-loop tracking anywhere near the actual disturbance band** -- see
CLAUDE.md for the real options going forward (most likely: the firmware
needs to generate the sine setpoint on-board, removing the host-command
bottleneck entirely, rather than the Pi/laptop trying to stream ~150+
individual setpoint commands per second over a link that provably tops
out around 3).

Usage:
  python3 fta_closed_loop_sine_response_test_vcp.py --freq HZ
      [--amplitude-px N] [--base-dac-y N] [--kp-milli N] [--ki-milli N]
      [--duration SEC] [--update-rate HZ] [--port PORT] [--out PATH]

    --freq              sine frequency in Hz -- required.
    --amplitude-px N    peak deviation of target_x from center, in pixels,
                        default 25 (matches the step-response tests'
                        small-step convention).
    --base-dac-y N      open-loop pre-position before engaging the loop,
                        default 2048 (this session's cleanest region).
    --kp-milli N        Kp * 1000, default 1750.
    --ki-milli N        Ki * 1000, default 200000 (the clean, ~141ms-
                        settling candidate from the 2026-08-13 tuning
                        pass -- see CLAUDE.md).
    --duration SEC      sine duration, default max(2.0, 8/freq).
    --update-rate HZ    REQUESTED target_x update rate, default 35 -- close
                        to the real measured ceiling (~38-39 commands/s,
                        2026-08-13, after raising the clock to 16MHz and
                        baud to 460800) at the 0.2ms/char pacing this
                        loop uses, not a free choice -- requesting higher
                        just means the pacing loop itself is the
                        bottleneck and the achieved rate stays ~38-39Hz
                        regardless (the loop self-limits, see the
                        "achieved update rate" it prints). >=10x --freq is
                        recommended for a non-staircased trajectory; at
                        this ceiling that comfortably covers freq<=3.5Hz
                        and gives partial (not ideal, but usable) coverage
                        up to ~10Hz -- still short of cleanly covering the
                        full 10-20Hz disturbance band, but a ~13x
                        improvement over the ~3Hz ceiling this project was
                        stuck at before that fix.
    --port PORT         Nucleo VCP serial port, default auto-detect.
    --out PATH          raw (t, x, cmd_t, cmd_v) npz path -- default
                        results/fta_closed_loop_sine_response_vcp_<freq>Hz_
                        <UTC timestamp>.npz. A PNG plot is saved alongside.

Requires the Pi to already be streaming telemetry.
"""
import argparse
import ctypes
import math
import re
import threading
import time
from datetime import datetime, timezone

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FTA_BAUD = 460800  # raised from 115200 back to 460800 on 2026-08-13, now
                    # matching the old "FTA Controller"'s rate again --
                    # needed the whole project's clock tree raised too
                    # (4MHz -> 16MHz HSI) since 460800 was unreachable at
                    # 4MHz. See CLAUDE.md for the full story, including
                    # real measured throughput at the new baud+clock: a
                    # paced ~1ms/char write is 100% reliable even under
                    # real multi-threaded contention (~39 commands/s), a
                    # dramatic improvement over the ~3Hz ceiling this
                    # module's docstring below was written against -- that
                    # docstring's throughput numbers are now stale.
MICRONS_PER_PIXEL = 3.0

REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+pkts=(\d+)\s+errs=(\d+)$")
STATUS_FIELD_RE = {
    "dac_x": re.compile(r"dac_x=(-?\d+)"),
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_x": re.compile(r"tel_x=(-?[\d.]+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
    "target_x": re.compile(r"target_x=(-?[\d.]+)"),
}

BLUE = "#2a78d6"
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


def send_command(ser, cmd, char_delay=0.02, reply_timeout=2.0):
    """Paced write, ~20ms/char -- for one-shot setup commands only, NOT
    the continuous target_x sine loop (see module docstring)."""
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


def get_status(ser, retries=5):
    for _ in range(retries):
        reply = send_command(ser, "get_status")
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
                "target_x": float(matches["target_x"].group(1)),
            }
    raise RuntimeError("No parseable get_status reply after several attempts.")


def fit_tracking(t, measured, target_center, target_amplitude, freq):
    """Linear least-squares fit of measured ~= A*sin(wt) + B*cos(wt) + C,
    w known exactly (we commanded it). Returns (gain, lag_ms, offset).
    gain is normalized by target_amplitude so 1.0 = perfect tracking
    (measured cx has exactly the commanded peak deviation); no sign
    handling needed -- see module docstring for why (target and measured
    share units/sign by construction, unlike the open-loop DAC-vs-pixel
    case fta_sine_response_test_vcp.py's fit_sine handles)."""
    w = 2.0 * math.pi * freq
    basis = np.stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)], axis=1)
    coeffs, *_ = np.linalg.lstsq(basis, measured, rcond=None)
    A, B, C = coeffs
    amplitude = float(np.hypot(A, B))
    phase = float(np.arctan2(B, A))  # lag relative to the commanded sin(wt), 0 phase by construction
    gain = amplitude / target_amplitude
    lag_ms = -phase / w * 1000.0
    return gain, lag_ms, float(C) - target_center


def _reader_thread(ser, t0, records, stop_event, reply_counts):
    """Drains telemetry AND this script's own set_target_x OK/ERR replies
    during the sine loop -- same rationale as the open-loop script's
    reader thread: without this, the firmware's blocking reply transmit
    could eventually stall waiting for host-side buffer space. Also counts
    OK vs ERR replies (reply_counts, a dict mutated in place) -- added
    2026-08-13 after a first real run found target_x never moved at all
    (100% of 1600 fire-and-forget updates silently dropped at 200Hz) --
    this makes the actual landing rate visible instead of inferring it
    indirectly from a zero tracking gain."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except Exception:
            continue
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        if line.startswith("OK target_x"):
            reply_counts["ok"] += 1
            continue
        if line.startswith("ERR"):
            reply_counts["err"] += 1
            continue
        now = time.monotonic() - t0
        m = TELEMETRY_RE.match(line)
        if not m:
            continue
        status = int(m.group(2))
        if not (status & 1):
            continue
        x = float(m.group(3))
        records.append((now, x))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--freq", type=float, required=True)
    parser.add_argument("--amplitude-px", type=float, default=25.0)
    parser.add_argument("--base-dac-y", type=int, default=2048)
    parser.add_argument("--kp-milli", type=int, default=1750)
    parser.add_argument("--ki-milli", type=int, default=200000)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--update-rate", type=float, default=35.0)
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    # Windows defaults to a ~15.6ms timer tick, silently inflating every
    # time.sleep(0.001) call in the paced-write loop below to ~15.6ms
    # instead of ~1ms -- measured directly, 2026-08-13 (50x sleep(0.001)
    # took 0.781s, not ~0.05s). This is what capped the first real run of
    # this script to ~4Hz achieved update rate against a requested 50Hz,
    # leaving nowhere near enough samples/cycle to trace a sine at all
    # (tracking gain came back ~0). Requesting 1ms system timer resolution
    # (a standard, common Windows technique) measured a 10x improvement
    # (~1.5ms actual per sleep(0.001) call). atexit guarantees
    # timeEndPeriod runs on every exit path (including the several
    # raise SystemExit(1) calls below) without restructuring this
    # function's existing control flow.
    import atexit
    winmm = ctypes.WinDLL("winmm")
    winmm.timeBeginPeriod(1)
    atexit.register(winmm.timeEndPeriod, 1)

    duration = args.duration if args.duration is not None else max(2.0, 8.0 / args.freq)
    if args.update_rate < 10 * args.freq:
        print(f"WARNING: --update-rate ({args.update_rate}) is less than 10x --freq "
              f"({args.freq}) -- the commanded target trajectory will look like a "
              "coarse staircase, not a smooth sine.")

    import serial

    port = args.port or find_fta_port()
    if port is None:
        print("No ST-Link serial port found -- pass --port explicitly.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    print(send_command(ser, "clear_estop"))
    print(send_command(ser, "set_mode open_loop"))

    st = get_status(ser)
    if st["tel_age_ms"] > 500:
        print(f"ERR: last relayed I2C telemetry is {st['tel_age_ms']}ms old -- "
              "nothing appears to be streaming from the Pi.")
        ser.close()
        raise SystemExit(1)

    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        print(send_command(ser, "amp_enable"))
        st = get_status(ser)
        if not st["amp"]:
            print("ERR: sent amp_enable but get_status still reports amp=off -- aborting.")
            ser.close()
            raise SystemExit(1)

    print(f"Pre-positioning dac_y={args.base_dac_y} (open_loop)...")
    print(send_command(ser, f"set_y {args.base_dac_y}"))
    time.sleep(0.5)

    st = get_status(ser)
    center_px = st["tel_x"]
    print(f"baseline cx={center_px:.1f}  amplitude={args.amplitude_px}px  "
          f"freq={args.freq}Hz  duration={duration:.2f}s  update_rate={args.update_rate}Hz  "
          f"Kp_milli={args.kp_milli} Ki_milli={args.ki_milli}")

    print(send_command(ser, f"set_target_x {round(center_px)}"))
    print(send_command(ser, f"set_kp {args.kp_milli}"))
    print(send_command(ser, f"set_ki {args.ki_milli}"))
    print(send_command(ser, "set_mode closed_loop"))
    time.sleep(0.3)

    records = []
    commanded = []
    reply_counts = {"ok": 0, "err": 0}
    stop_event = threading.Event()
    t0 = time.monotonic()
    reader = threading.Thread(target=_reader_thread, args=(ser, t0, records, stop_event, reply_counts), daemon=True)

    ser.reset_input_buffer()  # discard whatever backlogged during the 0.3s settle above
    reader.start()

    dt_cmd = 1.0 / args.update_rate
    n_samples = int(duration * args.update_rate)
    w = 2.0 * math.pi * args.freq
    for i in range(n_samples):
        t_cmd = i * dt_cmd
        value = round(center_px + args.amplitude_px * math.sin(w * t_cmd))
        # 0.2ms/char pacing, not a raw burst -- a raw burst write reliably
        # loses bytes regardless of baud/clock (confirmed again after the
        # 2026-08-13 clock+baud fix: 0% clean at true single-burst). The
        # original version of this loop used 20ms/char based on an
        # earlier same-day finding that turned out to be confounded by a
        # separate bug (Windows' default ~15.6ms timer tick silently
        # inflating every requested delay -- see main()'s
        # winmm.timeBeginPeriod(1) call and CLAUDE.md). After raising the
        # system clock to 16MHz and USART2 to 460800 baud (also
        # CLAUDE.md), a real measured back-to-back reliability test found
        # 0.1-0.5ms/char all give ~99-100% clean delivery at ~38-39
        # commands/s (throughput plateaus there regardless of further
        # lowering the delay -- per-write()-call overhead, not the sleep
        # itself, is the remaining bottleneck) -- a ~13x improvement over
        # the ~3Hz ceiling this project was stuck at before that fix.
        for ch in f"set_target_x {value}\n":
            ser.write(ch.encode("ascii"))
            time.sleep(0.0002)
        commanded.append((t_cmd, value))
        target_t = t0 + t_cmd + dt_cmd
        sleep_s = target_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)

    actual_span = time.monotonic() - t0
    time.sleep(0.3)
    stop_event.set()
    reader.join(timeout=1.0)

    print(send_command(ser, "set_mode open_loop"))
    print(send_command(ser, "set_y 95"))
    if not amp_was_enabled:
        print(send_command(ser, "amp_disable"))
    print("final:", get_status(ser))
    ser.close()

    print(f"Command loop took {actual_span:.3f}s for {n_samples} commands "
          f"(target {duration:.3f}s) -- achieved update rate "
          f"~{n_samples / actual_span:.0f}Hz.")
    landed = reply_counts["ok"] + reply_counts["err"]
    print(f"Update landing rate: {reply_counts['ok']} OK + {reply_counts['err']} ERR "
          f"replies seen out of {n_samples} attempts "
          f"({100*landed/n_samples:.1f}% landed, {100*reply_counts['ok']/n_samples:.1f}% applied).")

    if len(records) < 3 * args.freq * duration:
        print(f"Only {len(records)} usable telemetry samples over {duration:.2f}s -- "
              "may not be enough to fit a clean sine.")
        if len(records) < 6:
            return

    t = np.array([r[0] for r in records])
    x = np.array([r[1] for r in records])
    cmd_t = np.array([c[0] for c in commanded])
    cmd_v = np.array([c[1] for c in commanded], dtype=float)

    gain, lag_ms, offset = fit_tracking(t, x, center_px, args.amplitude_px, args.freq)
    period_ms = 1000.0 / args.freq
    lag_deg = (lag_ms / period_ms) * 360.0
    um = MICRONS_PER_PIXEL
    print(f"\ntracking gain: {gain:.3f} ({gain*100:.1f}% of commanded {args.amplitude_px}px "
          f"amplitude achieved, {gain*args.amplitude_px:.1f}px / {gain*args.amplitude_px*um:.1f}um)")
    print(f"lag: {lag_ms:.1f}ms ({lag_deg:.1f} deg at {args.freq}Hz)")
    print(f"offset from center: {offset:.2f}px ({offset*um:.1f}um)")
    print(f"implied disturbance-rejection sensitivity |S| = |1-T| ~= {abs(1-gain):.3f} "
          "(lower is better rejection; this is a magnitude-only approximation, not a full "
          "complex S=1-T since lag/phase isn't folded in here)")

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_path = args.out or f"results/fta_closed_loop_sine_response_vcp_{args.freq:g}Hz_{ts}.npz"
    np.savez(out_path, t=t, x=x, cmd_t=cmd_t, cmd_v=cmd_v,
              freq=args.freq, amplitude_px=args.amplitude_px, center_px=center_px,
              base_dac_y=args.base_dac_y, kp_milli=args.kp_milli, ki_milli=args.ki_milli,
              gain=gain, lag_ms=lag_ms, offset=offset)
    print(f"Saved raw time series to {out_path}")

    # --- plot ---
    fig, ax = plt.subplots(figsize=(10, 4.5), dpi=150)
    ax.set_facecolor("white")
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9, length=3)

    ax.plot(cmd_t, cmd_v, color=TARGET_COLOR, linewidth=1.1, linestyle=(0, (2, 2)), label="target_x (commanded)")
    ax.plot(t, x, color=BLUE, linewidth=1.3, label="measured cx")

    sec = ax.secondary_yaxis("right", functions=(lambda px: px * um, lambda v: v / um))
    sec.tick_params(colors=MUTED, labelsize=9, length=3)
    sec.set_ylabel("µm", fontsize=9, color=MUTED)

    ax.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
    ax.set_ylabel("cx (px)", fontsize=9.5, color=MUTED)
    ax.legend(frameon=False, fontsize=9, loc="upper right")

    parts = [f"{args.freq}Hz  amplitude={args.amplitude_px}px @ dac_y={args.base_dac_y}",
             f"Kp={args.kp_milli/1000:.2f} Ki={args.ki_milli/1000:.2f}",
             f"gain: {gain:.2f}  lag: {lag_ms:.1f}ms ({lag_deg:.0f}°)"]
    ax.text(0.02, 0.03, "\n".join(parts), transform=ax.transAxes, fontsize=8.5,
            color="#0b0b0b", va="bottom", ha="left",
            bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=4))

    fig.suptitle(f"Closed-loop sine tracking, dac_y → cx, {args.freq}Hz", fontsize=13, fontweight="bold")
    fig.tight_layout()
    png_path = out_path.rsplit(".", 1)[0] + ".png"
    fig.savefig(png_path, facecolor="white")
    plt.close(fig)
    print(f"Saved plot to {png_path}")


if __name__ == "__main__":
    main()
