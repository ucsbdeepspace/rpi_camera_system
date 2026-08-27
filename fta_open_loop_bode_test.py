#!/usr/bin/env python3
"""
Open-loop plant Bode measurement (dac_y -> cx), using the firmware's
on-board OPEN-LOOP sine generator (start_open_sine/stop_open_sine, added
2026-08-19) -- the direct analog of fta_closed_loop_onboard_sine_test.py,
but driving dac_y directly via apply_dac() instead of moving the PID's
target_x. No controller involved at all: this measures the actuator +
optics + flexure ("the plant") on its own, decoupled from every
controller-tuning question this project has spent today on.

Motivation (2026-08-19): after exhausting the controller-tuning space
(Kp has a hard low ceiling that never moves; Ki's ceiling doesn't move
with rate fixes, jitter fixes, or a resonance notch; D actively hurts
everywhere it's tried) and separately measuring the real closed-loop
delay as much smaller than assumed (~11.5ms, not ~41ms -- see
fta_loop_delay_test.py), the natural next question is whether this is a
phase-margin/compensator-design problem or a hard mechanical bandwidth
ceiling. Answering that needs a real open-loop frequency response
(magnitude AND phase) across the band, not another closed-loop trial.

Measurement: dac_y is commanded as dac_y(t) = center + amplitude*sin(2*pi*f*t)
directly by the firmware. The relay telemetry line already reports the
REAL applied dac_y (g_last_dac_y, via apply_dac()) alongside measured cx,
both timestamped by the firmware's own tick= (HAL_GetTick(), immune to
host clock jitter -- see fta_closed_loop_onboard_sine_test.py's own
2026-08-19 fix for why host arrival timestamps can't be trusted for this).
Fitting BOTH traces against the same sin(wt)/cos(wt) basis and taking the
magnitude ratio / phase difference gives a real open-loop Bode point --
gain in px/count (a physical unit, not a % of commanded amplitude like
the closed-loop tracking-gain tests), phase in degrees -- with no need to
trust the host's t=0 or the firmware's exact realized amplitude/center
(same "fit both, diff cancels" trick as fit_tracking() in the closed-loop
script, reused verbatim here).

Usage:
  python3 fta_open_loop_bode_test.py --freq HZ [--amplitude-counts N]
      [--base-dac-y N] [--duration SEC] [--port PORT] [--out PATH]

  python3 fta_open_loop_bode_test.py --sweep "1,2,3,5,7,10,13,15,18,20,25,30,35,38.5,42,50"
      [--amplitude-counts N] [--base-dac-y N] [--out-prefix PATH]
      runs the whole list in one process (one connection), saving one
      npz+png per frequency plus a combined summary plot at the end.
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
MICRONS_PER_PIXEL = 3.0  # OV9281 pixel pitch, same constant used throughout this project.

REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+tick=(\d+)\s+pkts=(\d+)\s+errs=(\d+)$")
# Core fields only required for the connectivity/amp check -- open_sine
# fields are read separately with retries, same reasoning as the
# 2026-08-19 fix in fta_closed_loop_onboard_sine_test.py (a corrupted/
# dropped trailing field must not silently read back as a false 0).
STATUS_FIELD_RE = {
    "dac_x": re.compile(r"dac_x=(-?\d+)"),
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_x": re.compile(r"tel_x=(-?[\d.]+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
    "open_sine": re.compile(r"open_sine=(\d+)"),
    "open_sine_freq_millihz": re.compile(r"open_sine_freq_millihz=(-?\d+)"),
}
CORE_STATUS_FIELDS = ("dac_x", "dac_y", "amp", "tel_x", "tel_age_ms")
ALL_STATUS_FIELDS = tuple(STATUS_FIELD_RE.keys())

BLUE = "#2a78d6"
ORANGE = "#eb6834"
MUTED = "#898781"
GRID = "#e1e0d9"


def find_fta_port():
    from serial.tools import list_ports
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    return candidates[0].device if candidates else None


def send_command(ser, cmd, char_delay=0.02, reply_timeout=2.0, retries=1):
    for _ in range(retries):
        ser.reset_input_buffer()
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


def get_status(ser, retries=5, required=ALL_STATUS_FIELDS):
    for _ in range(retries):
        reply = send_command(ser, "get_status", retries=1)
        if reply is None or not reply.startswith("STATUS"):
            continue
        matches = {k: rx.search(reply) for k, rx in STATUS_FIELD_RE.items()}
        if all(matches[k] for k in required):
            os_m = matches["open_sine"]
            osf_m = matches["open_sine_freq_millihz"]
            return {
                "dac_x": int(matches["dac_x"].group(1)),
                "dac_y": int(matches["dac_y"].group(1)),
                "amp": int(matches["amp"].group(1)),
                "tel_x": float(matches["tel_x"].group(1)),
                "tel_age_ms": int(matches["tel_age_ms"].group(1)),
                "open_sine": int(os_m.group(1)) if os_m else 0,
                "open_sine_freq_millihz": int(osf_m.group(1)) if osf_m else 0,
            }
    raise RuntimeError("No parseable get_status reply after several attempts.")


def fit_sine_component(t, y, w):
    basis = np.stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)], axis=1)
    coeffs, *_ = np.linalg.lstsq(basis, y, rcond=None)
    A, B, C = coeffs
    return float(A), float(B), float(C)


def fit_bode_point(t, measured_x, commanded_dac_y, freq):
    """Same 'fit both against the same basis, difference cancels any t0
    error' trick as fit_tracking() in the closed-loop onboard sine
    script -- here the 'reference' trace is the REPORTED dac_y (real
    plant input), not a reconstructed target, and the gain is a genuine
    physical units ratio (px measured per DAC count commanded), not a
    percentage of commanded amplitude."""
    w = 2.0 * math.pi * freq
    Ax, Bx, Cx = fit_sine_component(t, measured_x, w)
    Ad, Bd, Cd = fit_sine_component(t, commanded_dac_y, w)
    amp_x = float(np.hypot(Ax, Bx))
    amp_d = float(np.hypot(Ad, Bd))
    phase_x = float(np.arctan2(Bx, Ax))
    phase_d = float(np.arctan2(Bd, Ad))
    gain_px_per_count = amp_x / amp_d if amp_d > 1e-6 else float("nan")
    phase_diff = (phase_x - phase_d + math.pi) % (2.0 * math.pi) - math.pi
    lag_ms = -phase_diff / w * 1000.0
    lag_deg = -math.degrees(phase_diff)
    return {
        "gain_px_per_count": gain_px_per_count,
        "amp_x_px": amp_x, "amp_dac_counts": amp_d,
        "lag_ms": lag_ms, "lag_deg": lag_deg,
        # Fit coefficients, kept so a plot can overlay the actual fitted
        # curve (not just the raw scatter) for visual confirmation the
        # least-squares fit is trustworthy -- matters most right around
        # the resonance, where the raw trace is least sinusoidal-looking.
        "fit_Ax": Ax, "fit_Bx": Bx, "fit_Cx": Cx,
        "fit_Ad": Ad, "fit_Bd": Bd, "fit_Cd": Cd,
    }


def _reader_thread(ser, records, stop_event):
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except Exception:
            continue
        if not raw:
            continue
        m = TELEMETRY_RE.match(raw.decode(errors="replace").strip())
        if not m:
            continue
        status = int(m.group(2))
        if not (status & 1):
            continue
        x = float(m.group(3))
        dac_y = int(m.group(6))
        tick_ms = int(m.group(7))
        records.append((tick_ms, x, dac_y))


def emergency_cleanup(ser):
    for cmd in ("stop_open_sine", "set_mode open_loop", "set_y 95", "amp_disable"):
        try:
            send_command(ser, cmd)
        except Exception:
            pass


def run_one_frequency(ser, freq, amplitude_counts, base_dac_y, duration, amp_was_enabled):
    """Runs a single Bode point. Returns a dict of results, or None on a
    hard failure (already left hardware idle in that case)."""
    print(f"\n--- {freq:.2f} Hz ---")
    send_command(ser, f"set_y {base_dac_y}")
    time.sleep(0.4)

    freq_millihz = round(freq * 1000)
    reply = send_command(ser, f"start_open_sine {freq_millihz} {amplitude_counts} {base_dac_y}")
    print(reply)

    # Ground-truth check (same reasoning as fta_closed_loop_onboard_sine_test.py
    # 2026-08-19: a lost reply doesn't mean the command didn't land -- only
    # treat it as a real failure if get_status ALSO disagrees).
    try:
        verify_st = get_status(ser, retries=10)
        confirmed = bool(verify_st["open_sine"]) and verify_st["open_sine_freq_millihz"] == freq_millihz
    except RuntimeError:
        print("WARNING: get_status failed to verify start_open_sine -- proceeding anyway.")
        confirmed = True
    if not confirmed:
        print(f"ERR: start_open_sine not confirmed (open_sine={verify_st['open_sine']} "
              f"freq={verify_st['open_sine_freq_millihz']}, expected {freq_millihz}) -- skipping this point.")
        send_command(ser, "stop_open_sine")
        return None

    records = []
    stop_event = threading.Event()
    reader = threading.Thread(target=_reader_thread, args=(ser, records, stop_event), daemon=True)
    reader.start()
    time.sleep(duration)
    stop_event.set()
    reader.join(timeout=1.0)

    send_command(ser, "stop_open_sine")
    send_command(ser, f"set_y {base_dac_y}")

    if len(records) < 10:
        print(f"  only {len(records)} samples, skipping")
        return None

    tick_ms = np.array([r[0] for r in records], dtype=np.float64)
    x = np.array([r[1] for r in records])
    dac_y = np.array([r[2] for r in records], dtype=np.float64)
    t = (tick_ms - tick_ms[0]) / 1000.0

    fit = fit_bode_point(t, x, dac_y, freq)
    print(f"  {len(records)} samples over {t[-1]:.2f}s (~{len(records)/t[-1]:.0f}/s)  "
          f"gain={fit['gain_px_per_count']:.4f} px/count  lag={fit['lag_ms']:.1f}ms "
          f"({fit['lag_deg']:.1f} deg)")

    return {"freq": freq, "t": t, "x": x, "dac_y": dac_y, **fit}


def save_point_plot(result, amplitude_counts, base_dac_y, out_path):
    """2x2 grid: measured cx (top) and commanded dac_y (bottom), each as a
    LEFT full-duration view (context) and a RIGHT zoomed first-~6-cycle
    view (so individual cycles are actually distinguishable by eye) --
    both panels show the raw telemetry as scattered points AND the actual
    least-squares fitted sinusoid overlaid as a solid line, for visual
    fit confirmation. The full-duration view alone becomes an unreadable
    wall of overlapping cycles above ~10-15Hz (confirmed directly: the
    38.5Hz point crams 84+ cycles into one panel) -- the zoomed view is
    what actually lets a fit be checked by eye at those frequencies."""
    t, x, dac_y = result["t"], result["x"], result["dac_y"]
    freq = result["freq"]
    w = 2.0 * math.pi * freq
    t_fit = np.linspace(t[0], t[-1], 2000)
    x_fit = (result["fit_Ax"] * np.sin(w * t_fit) + result["fit_Bx"] * np.cos(w * t_fit)
             + result["fit_Cx"])
    d_fit = (result["fit_Ad"] * np.sin(w * t_fit) + result["fit_Bd"] * np.cos(w * t_fit)
             + result["fit_Cd"])

    zoom_end = min(t[-1], t[0] + 6.0 / freq)
    zoom_mask = t <= zoom_end
    zoom_fit_mask = t_fit <= zoom_end

    fig, axes = plt.subplots(2, 2, figsize=(11.5, 6.8), dpi=150,
                              gridspec_kw={"width_ratios": [1.5, 1]})
    (ax1, ax1z), (ax2, ax2z) = axes
    for a in axes.flat:
        a.set_facecolor("white")
        for spine in ("top", "right"):
            a.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            a.spines[spine].set_color(GRID)
        a.tick_params(colors=MUTED, labelsize=8.5, length=3)

    for a, az, raw, fit_curve, color, fit_color, ylabel in (
        (ax1, ax1z, x, x_fit, BLUE, "#1a4d8f", "measured cx (px)"),
        (ax2, ax2z, dac_y, d_fit, ORANGE, "#a8480f", "commanded dac_y (counts)"),
    ):
        a.scatter(t, raw, color=color, s=8, alpha=0.45, linewidths=0)
        a.plot(t_fit, fit_curve, color=fit_color, linewidth=1.3)
        a.set_ylabel(ylabel, fontsize=9.5, color=MUTED)

        az.scatter(t[zoom_mask], raw[zoom_mask], color=color, s=16, alpha=0.6, linewidths=0)
        az.plot(t_fit[zoom_fit_mask], fit_curve[zoom_fit_mask], color=fit_color, linewidth=1.8)

    ax1.set_title("full duration", fontsize=9, color=MUTED, loc="left")
    ax1z.set_title("zoomed (first ~6 cycles)", fontsize=9, color=MUTED, loc="left")
    ax2.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
    ax2z.set_xlabel("time (s)", fontsize=9.5, color=MUTED)

    fig.suptitle(f"{freq:.2f} Hz open-loop plant excitation -- "
                 f"gain={result['gain_px_per_count']:.4f} px/count, "
                 f"lag={result['lag_ms']:.1f}ms ({result['lag_deg']:.1f} deg)",
                 fontsize=11.5, color="#1A1A2E", x=0.02, ha="left")

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path, facecolor="white")
    plt.close(fig)


def save_bode_summary(results, out_path):
    """Gain is reported as um (real beam displacement) per DAC count
    commanded -- NOT pixels, which aren't a physically meaningful unit to
    anyone reading this outside the camera pipeline. Uses
    MICRONS_PER_PIXEL (the OV9281's real pixel pitch, confirmed live
    elsewhere in this project via UnitCellSize) to convert the raw
    px/count fit result -- same conversion this project already applies
    everywhere else displacement is reported (e.g.
    fta_closed_loop_step_response_vcp.py).

    Phase is unwrapped across the sorted-by-frequency sequence
    (np.unwrap) rather than left at each point's raw atan2 wrap -- the
    fundamental single-frequency wraparound ambiguity this project has
    hit before (can't distinguish a lag from lag +/- n*360 deg from one
    tone alone), resolved here by the data's own smooth, monotonic
    progression rather than left aliased past +/-180 deg, which would
    otherwise show as a nonsensical jump right at the resonance."""
    order = np.argsort([r["freq"] for r in results])
    freqs = np.array([results[i]["freq"] for i in order])
    gains_um = np.array([results[i]["gain_px_per_count"] * MICRONS_PER_PIXEL for i in order])
    lag_rad = np.deg2rad([results[i]["lag_deg"] for i in order])
    lag_unwrapped_deg = np.rad2deg(np.unwrap(lag_rad))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.5, 7.5), dpi=150, sharex=True)
    for a in (ax1, ax2):
        a.set_facecolor("white")
        for spine in ("top", "right"):
            a.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            a.spines[spine].set_color(GRID)
        a.tick_params(colors=MUTED, labelsize=9, length=3)
        a.set_xscale("log")
        a.grid(True, which="both", color=GRID, linewidth=0.6)

    ax1.plot(freqs, gains_um, color=BLUE, marker="o", linewidth=1.8, markersize=5)
    ax1.set_ylabel("open-loop gain (µm / DAC count)", fontsize=9.5, color=MUTED)
    ax1.set_title("Open-loop plant Bode plot (dac_y -> beam displacement)",
                   fontsize=12, color="#1A1A2E", loc="left")
    ax1.axvline(38.5, color=MUTED, linewidth=1.0, linestyle=(0, (2, 2)))
    ax1.text(38.5, max(gains_um) * 0.95, " 38.5Hz\n resonance", fontsize=8, color=MUTED)
    if freqs.min() <= 20 <= freqs.max() and freqs.min() <= 10 <= freqs.max():
        ax1.axvspan(10, 20, color=BLUE, alpha=0.07)
        ax2.axvspan(10, 20, color=BLUE, alpha=0.07)
        ax1.text(11, gains_um.min() + 0.02 * (gains_um.max() - gains_um.min()),
                  "10-20Hz\ntarget band", fontsize=8, color=BLUE)

    ax2.plot(freqs, lag_unwrapped_deg, color=ORANGE, marker="s", linewidth=1.8, markersize=5)
    ax2.axhline(-90, color=MUTED, linewidth=0.8, linestyle=(0, (1, 2)))
    ax2.axhline(-180, color=MUTED, linewidth=0.8, linestyle=(0, (1, 2)))
    ax2.set_ylabel("phase (deg, unwrapped)", fontsize=9.5, color=MUTED)
    ax2.set_xlabel("frequency (Hz)", fontsize=9.5, color=MUTED)
    ax2.axvline(38.5, color=MUTED, linewidth=1.0, linestyle=(0, (2, 2)))

    fig.tight_layout()
    fig.savefig(out_path, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--freq", type=float, default=None)
    parser.add_argument("--sweep", type=str, default=None,
                         help="comma-separated list of Hz values, run in one connection")
    parser.add_argument("--amplitude-counts", type=int, default=300)
    parser.add_argument("--base-dac-y", type=int, default=2048)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    parser.add_argument("--out-prefix", default="results/fta_open_loop_bode")
    args = parser.parse_args()

    if args.freq is None and args.sweep is None:
        parser.error("--freq or --sweep is required")

    freqs = [args.freq] if args.freq is not None else [float(s) for s in args.sweep.split(",")]

    import serial
    port = args.port or find_fta_port()
    if port is None:
        print("No ST-Link serial port found.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {FTA_BAUD}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()

    send_command(ser, "clear_estop")
    send_command(ser, "set_mode open_loop")

    st = get_status(ser, required=CORE_STATUS_FIELDS)
    if st["tel_age_ms"] > 500:
        print(f"ERR: last relayed telemetry is {st['tel_age_ms']}ms old -- nothing streaming from the Pi.")
        ser.close()
        raise SystemExit(1)

    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        send_command(ser, "amp_enable")
        st = get_status(ser, required=CORE_STATUS_FIELDS)
        if not st["amp"]:
            print("ERR: amp_enable didn't take -- aborting.")
            ser.close()
            raise SystemExit(1)

    results = []
    try:
        for freq in freqs:
            duration = args.duration if args.duration is not None else max(2.0, 8.0 / freq)
            result = run_one_frequency(ser, freq, args.amplitude_counts, args.base_dac_y,
                                        duration, amp_was_enabled)
            if result is None:
                continue
            ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            if args.out and len(freqs) == 1:
                out_path = args.out
            else:
                out_path = f"{args.out_prefix}_{freq:g}Hz_{ts}.npz"
            np.savez(out_path, t=result["t"], x=result["x"], dac_y=result["dac_y"],
                     freq=freq, amplitude_counts=args.amplitude_counts, base_dac_y=args.base_dac_y,
                     gain_px_per_count=result["gain_px_per_count"], lag_ms=result["lag_ms"],
                     lag_deg=result["lag_deg"],
                     fit_Ax=result["fit_Ax"], fit_Bx=result["fit_Bx"], fit_Cx=result["fit_Cx"],
                     fit_Ad=result["fit_Ad"], fit_Bd=result["fit_Bd"], fit_Cd=result["fit_Cd"])
            png_path = out_path.rsplit(".", 1)[0] + ".png"
            save_point_plot(result, args.amplitude_counts, args.base_dac_y, png_path)
            print(f"  saved {out_path}")
            results.append(result)
    finally:
        emergency_cleanup(ser)
        ser.close()

    if len(results) >= 3:
        summary_path = f"{args.out_prefix}_summary.png"
        save_bode_summary(results, summary_path)
        print(f"\nSaved Bode summary to {summary_path}")

    print("\n=== Bode results ===")
    print(f"{'freq (Hz)':>10} {'gain (px/count)':>16} {'lag (ms)':>10} {'lag (deg)':>10}")
    for r in results:
        print(f"{r['freq']:>10.2f} {r['gain_px_per_count']:>16.4f} {r['lag_ms']:>10.1f} {r['lag_deg']:>10.1f}")


if __name__ == "__main__":
    main()
