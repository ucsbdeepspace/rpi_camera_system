#!/usr/bin/env python3
"""
Free-decay (impulse/step-release) resonance test -- independent of the
closed-loop controller and camera-rate limitations that make the fitted
sine-tracking gain/lag numbers hard to fully trust as a resonance
measurement. Idea (user, 2026-08-18): with the amp OFF, the DAC output
has no physical effect (the amplifier stage is what actually drives the
voice coil) -- pre-load a DAC offset, then briefly pulse the amp ON (a
sudden step force is applied) and back OFF (drive is removed), and watch
the mechanical system ring down under its own free dynamics. This gives
a direct measurement of natural frequency/damping, unconfounded by
control-loop gains or telemetry-rate-limited phase-fitting.

Usage:
  python3 fta_ringdown_test.py [--offset-dac N] [--base-dac-y N]
      [--pulse-ms N] [--record-s N] [--port PORT] [--out PATH]
"""
import argparse
import re
import threading
import time
from datetime import datetime, timezone

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

FTA_BAUD = 460800
MICRONS_PER_PIXEL = 3.0

REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+pkts=(\d+)\s+errs=(\d+)$")

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


def send_command(ser, cmd, char_delay=0.02, reply_timeout=2.0, retries=5):
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
        y = float(m.group(4))
        records.append((now, x, y))


def damped_sine(t, amp, freq_hz, zeta, phase, offset, t0):
    """amp * exp(-zeta*2*pi*freq*(t-t0)) * cos(2*pi*freq*sqrt(1-zeta^2)*(t-t0) + phase) + offset,
    zero for t < t0 handled by caller (only fit t >= t0)."""
    wn = 2.0 * np.pi * freq_hz
    wd = wn * np.sqrt(max(1.0 - zeta ** 2, 1e-6))
    dt = t - t0
    return amp * np.exp(-zeta * wn * dt) * np.cos(wd * dt + phase) + offset


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base-dac-y", type=int, default=2048)
    parser.add_argument("--offset-dac", type=int, default=450,
                         help="DAC counts to jump to when the amp pulses on, default 450 "
                              "(matches this project's established small-step regime)")
    parser.add_argument("--pulse-ms", type=int, default=80,
                         help="how long to leave the amp on before cutting it, default 80ms")
    parser.add_argument("--record-s", type=float, default=2.0)
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

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
    print(send_command(ser, "amp_disable"))  # amp starts OFF -- set_y below has no physical effect yet
    print(send_command(ser, f"set_y {args.base_dac_y}"))
    time.sleep(0.3)

    records = []
    stop_event = threading.Event()
    t0 = time.monotonic()
    reader = threading.Thread(target=_reader_thread, args=(ser, t0, records, stop_event), daemon=True)
    ser.reset_input_buffer()
    reader.start()

    time.sleep(0.3)  # baseline hold, amp off, at base_dac_y (physically wherever it last settled)

    # Pre-load the step target while amp is still off (no physical effect yet),
    # then pulse the amp on -- the coil suddenly sees the full step, a real
    # impulsive force -- and back off, letting it ring down freely.
    #
    # Deliberately NOT using send_command() here (which reads replies) --
    # the reader thread above already owns ser.readline() for the whole
    # recording window. A second thread also calling readline() races for
    # incoming bytes, so send_command's own reply would frequently get
    # stolen by the reader thread (which silently drops anything that
    # doesn't match TELEMETRY_RE) -- exactly the mistake this project's
    # fta_closed_loop_step_response_vcp.py already flags in its own
    # docstring, hit for real on this script's first run: all three
    # send_command calls here exhausted their full retry budget (~10s
    # each) waiting for a reply that the reader thread kept stealing,
    # stretching an intended ~80ms pulse into ~24 real seconds. Paced
    # writes only, no reply read, matching that script's established
    # pattern for anything sent during an active recording window.
    def paced_write(cmd, char_delay=0.02):
        for ch in cmd + "\n":
            ser.write(ch.encode("ascii"))
            time.sleep(char_delay)

    pulse_target = args.base_dac_y + args.offset_dac
    paced_write(f"set_y {pulse_target}")
    t_pulse_on = time.monotonic() - t0
    paced_write("amp_enable")
    time.sleep(args.pulse_ms / 1000.0)
    paced_write("amp_disable")
    t_pulse_off = time.monotonic() - t0

    time.sleep(args.record_s)

    stop_event.set()
    reader.join(timeout=1.0)

    # Reply-reading is safe again now that the reader thread has stopped.
    print(send_command(ser, "set_y 95"))
    ser.close()

    print(f"Captured {len(records)} telemetry samples "
          f"(pulse on at {t_pulse_on*1000:.0f}ms, off at {t_pulse_off*1000:.0f}ms).")
    if len(records) < 20:
        print("Not enough samples to analyze.")
        return

    t = np.array([r[0] for r in records])
    x = np.array([r[1] for r in records])
    y = np.array([r[2] for r in records])

    # Fit the decay starting shortly after the amp turns off (avoid the
    # amp-on transient itself, which isn't the free-decay portion).
    fit_mask = t > (t_pulse_off + 0.01)
    t_fit = t[fit_mask]
    x_fit = x[fit_mask]

    freq_guess = 11.0  # this project's previously-documented open-loop resonance estimate
    amp_guess = (x_fit.max() - x_fit.min()) / 2.0
    offset_guess = x_fit[-20:].mean()
    p0 = [amp_guess, freq_guess, 0.05, 0.0, offset_guess, t_pulse_off]
    try:
        popt, pcov = curve_fit(
            lambda tt, amp, freq_hz, zeta, phase, offset: damped_sine(tt, amp, freq_hz, zeta, phase, offset, t_pulse_off),
            t_fit, x_fit, p0=p0[:5], maxfev=20000,
            bounds=([0, 1, 0.001, -np.pi, -1e6], [1e4, 60, 2.0, np.pi, 1e6]))
        amp_fit, freq_fit, zeta_fit, phase_fit, offset_fit = popt
        fit_ok = True
    except Exception as e:
        print(f"Damped-sine fit failed: {e}")
        fit_ok = False

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_path = args.out or f"results/fta_ringdown_{ts}.npz"
    save_kwargs = dict(t=t, x=x, y=y, t_pulse_on=t_pulse_on, t_pulse_off=t_pulse_off,
                        base_dac_y=args.base_dac_y, offset_dac=args.offset_dac, pulse_ms=args.pulse_ms)
    if fit_ok:
        save_kwargs.update(freq_hz=freq_fit, zeta=zeta_fit, amp_px=amp_fit)
    np.savez(out_path, **save_kwargs)
    print(f"Saved raw time series to {out_path}")

    if fit_ok:
        print(f"\nFitted free-decay: freq={freq_fit:.2f}Hz  damping ratio (zeta)={zeta_fit:.3f}  "
              f"amplitude={amp_fit:.2f}px ({amp_fit*MICRONS_PER_PIXEL:.1f}um)")
    else:
        print("\nFit failed -- inspect the plot manually.")

    # --- plot ---
    fig, ax = plt.subplots(figsize=(10, 4.5), dpi=150)
    ax.set_facecolor("white")
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9, length=3)

    ax.plot(t, x, color=BLUE, linewidth=1.2, label="measured cx")
    ax.axvline(t_pulse_on, color=MUTED, linewidth=0.8, linestyle=(0, (1, 2)), label="amp on")
    ax.axvline(t_pulse_off, color=ORANGE, linewidth=0.8, linestyle=(0, (1, 2)), label="amp off")
    if fit_ok:
        t_plot = np.linspace(t_pulse_off, t[-1], 500)
        fit_curve = damped_sine(t_plot, amp_fit, freq_fit, zeta_fit, phase_fit, offset_fit, t_pulse_off)
        ax.plot(t_plot, fit_curve, color="#2a2a2a", linewidth=1.0, linestyle=(0, (4, 2)),
                label=f"fit: {freq_fit:.1f}Hz, zeta={zeta_fit:.3f}")

    sec = ax.secondary_yaxis("right", functions=(lambda px: px * MICRONS_PER_PIXEL, lambda v: v / MICRONS_PER_PIXEL))
    sec.tick_params(colors=MUTED, labelsize=9, length=3)
    sec.set_ylabel("µm", fontsize=9, color=MUTED)

    ax.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
    ax.set_ylabel("cx (px)", fontsize=9.5, color=MUTED)
    ax.legend(frameon=False, fontsize=8.5, loc="upper right")

    fig.suptitle("Free-decay ring-down test (amp pulsed, then cut)", fontsize=13, fontweight="bold")
    fig.tight_layout()
    png_path = out_path.rsplit(".", 1)[0] + ".png"
    fig.savefig(png_path, facecolor="white")
    plt.close(fig)
    print(f"Saved plot to {png_path}")


if __name__ == "__main__":
    main()
