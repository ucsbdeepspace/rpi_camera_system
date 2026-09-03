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
  python3 fta_ringdown_test.py [--axis x|y] [--offset-dac N] [--base-dac N]
      [--pulse-ms N] [--record-s N] [--port PORT] [--out PATH]

--axis x (default): pulses dac_y, watches cx -- the primary axis
    (dac_y->cx), matching every ring-down measurement this project has
    made before (the ~38.5Hz resonance already on record).
--axis y: pulses dac_x, watches cy -- the second axis (dac_x->cy),
    never ring-down-tested before. Locked-optics calibration already
    found this pathway has both a different gain MAGNITUDE and OPPOSITE
    SIGN from the primary axis (dac_x->cy: -0.104 px/count vs.
    dac_y->cx: +0.126 px/count) -- no reason to assume it shares the
    primary axis's resonance either.
"""
import argparse
import re
import threading
import time
from datetime import datetime, timezone

import numpy as np
from scipy.signal import find_peaks
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

FTA_BAUD = 460800
MICRONS_PER_PIXEL = 3.0

REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
# dac_x= added 2026-09-03 (see the cross-axis coupling test) -- same
# recurring "add a wire field, forget the other consuming script's regex"
# bug class this project has hit repeatedly; this script had gone
# completely silent (0 samples matched, no error) until this was fixed.
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+dac_x=(-?\d+)\s+tick=(\d+)\s+pkts=(\d+)\s+errs=(\d+)\s+cseq=(\d+)$")

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


def _reader_thread(ser, records, stop_event):
    """Records the FIRMWARE's own tick (HAL_GetTick(), ms) per sample, not
    a host-side arrival timestamp -- see main()'s docstring on why host
    timestamps from this thread are unusable for fine-grained timing
    (Windows thread-scheduling granularity batches readline() calls into
    ~15-16ms bursts, confirmed directly, unaffected by
    winmm.timeBeginPeriod(1))."""
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
        y = float(m.group(4))
        tick_ms = int(m.group(8))
        records.append((tick_ms, x, y))


def damped_sine(t, amp, freq_hz, zeta, phase, offset, t0):
    """amp * exp(-zeta*2*pi*freq*(t-t0)) * cos(2*pi*freq*sqrt(1-zeta^2)*(t-t0) + phase) + offset,
    zero for t < t0 handled by caller (only fit t >= t0)."""
    wn = 2.0 * np.pi * freq_hz
    wd = wn * np.sqrt(max(1.0 - zeta ** 2, 1e-6))
    dt = t - t0
    return amp * np.exp(-zeta * wn * dt) * np.cos(wd * dt + phase) + offset


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--axis", choices=["x", "y"], default="x",
                         help="x (default): pulse dac_y, watch cx (primary axis). "
                              "y: pulse dac_x, watch cy (second axis).")
    parser.add_argument("--base-dac", type=int, default=2048,
                         help="idle DAC position for whichever axis is being pulsed "
                              "(was --base-dac-y; renamed since it now applies to either axis)")
    parser.add_argument("--offset-dac", type=int, default=450,
                         help="DAC counts to jump to when the amp pulses on, default 450 "
                              "(matches this project's established small-step regime)")
    parser.add_argument("--pulse-ms", type=int, default=80,
                         help="how long to leave the amp on before cutting it, default 80ms")
    parser.add_argument("--record-s", type=float, default=2.0)
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    pulse_cmd = "set_y" if args.axis == "x" else "set_x"
    signal_label = "cx" if args.axis == "x" else "cy"

    # Windows' default ~15.6ms system timer tick doesn't just inflate
    # time.sleep() (already found and fixed elsewhere this project, see
    # fta_closed_loop_sine_response_test_vcp.py) -- it also coarsens
    # thread-scheduling granularity, which silently batched this script's
    # _reader_thread's readline() calls into ~15-16ms buckets, discovered
    # by re-examining a saved run's own t[] array (69-86% of consecutive
    # samples shared an identical timestamp). For a ~15-65ms-period
    # oscillation, that's only ~1-4 real timestamp buckets per cycle --
    # nowhere near enough for a trustworthy frequency fit, despite the
    # underlying telemetry itself arriving at up to ~465Hz. Same standard
    # Windows high-resolution-timer request as the sine-test script.
    import atexit
    import ctypes
    winmm = ctypes.WinDLL("winmm")
    winmm.timeBeginPeriod(1)
    atexit.register(winmm.timeEndPeriod, 1)

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
    print(send_command(ser, "amp_disable"))  # amp starts OFF -- pulse_cmd below has no physical effect yet
    print(send_command(ser, f"{pulse_cmd} {args.base_dac}"))
    time.sleep(0.3)

    records = []
    stop_event = threading.Event()
    reader = threading.Thread(target=_reader_thread, args=(ser, records, stop_event), daemon=True)
    ser.reset_input_buffer()
    reader.start()

    t0 = time.monotonic()  # host time, only used for the human-readable pulse-timing print below
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

    pulse_target = args.base_dac + args.offset_dac
    paced_write(f"{pulse_cmd} {pulse_target}")
    t_pulse_on = time.monotonic() - t0
    paced_write("amp_enable")
    time.sleep(args.pulse_ms / 1000.0)
    paced_write("amp_disable")
    t_pulse_off = time.monotonic() - t0

    time.sleep(args.record_s)

    stop_event.set()
    reader.join(timeout=1.0)

    # Reply-reading is safe again now that the reader thread has stopped.
    print(send_command(ser, f"{pulse_cmd} 95"))
    ser.close()

    print(f"Captured {len(records)} telemetry samples "
          f"(host-approximate: pulse on at {t_pulse_on*1000:.0f}ms, off at {t_pulse_off*1000:.0f}ms "
          f"-- see below for the data-driven tick-based pulse-off estimate actually used for the fit).")
    if len(records) < 20:
        print("Not enough samples to analyze.")
        return

    tick_ms = np.array([r[0] for r in records], dtype=np.float64)
    t = (tick_ms - tick_ms[0]) / 1000.0  # firmware-clock seconds, relative to first sample
    x_raw = np.array([r[1] for r in records])
    y_raw = np.array([r[2] for r in records])
    x = x_raw if args.axis == "x" else y_raw  # the signal actually analyzed below -- named
                                                # "x" throughout (matching the original
                                                # x-axis-only script) regardless of which
                                                # physical telemetry field it came from

    # Locate a trustworthy fit-window start on the firmware-tick timeline.
    # Two earlier approaches both failed once real (not host-timestamp-
    # bucketed) resolution revealed the amp is actually driven for the
    # FULL host-paced amp_enable-to-amp_disable duration (~500-800ms in
    # practice, not the nominal --pulse-ms=80ms -- paced command
    # transmission dominates, same finding as the negative-lag t0 bug
    # elsewhere in this project) -- long enough that a real, forced
    # oscillation happens DURING that driven window, with drops
    # comparable in size to the true post-amp-off one. Neither "biggest
    # single-sample drop" nor "host-measured elapsed time" reliably
    # landed past the end of that forced portion; both fed curve_fit a
    # mixed forced+free window that converged on the slow envelope
    # instead of the real fast free-decay oscillation (visible by eye
    # once plotted, a much bigger/cleaner swing after everything else).
    #
    # Robust instead: the GLOBAL MINIMUM of x is unambiguous -- nothing
    # can drive the system past that trough once the amp is genuinely
    # off, so it must be part of the real free decay, not the forced
    # portion. Back up ~60ms (comfortably more than half a period at any
    # plausible frequency this rig has shown, 15-40Hz) to land on the
    # peak immediately preceding that trough, guaranteeing the fit window
    # captures the decay's largest, cleanest swing.
    # Plot-annotation marker ONLY (not used by the actual fit-window/peak-
    # spacing anchor below, which uses argmin regardless of sign -- see
    # that logic's own comment). argmax(x) assumed the amp-on-driven
    # excursion is always a RISE, true for the primary axis (dac_y->cx,
    # positive gain) but wrong for the second axis (dac_x->cy, NEGATIVE
    # gain per the locked-optics calibration -- CLAUDE.md 2026-08-12) --
    # driving harder there pushes the signal DOWN, so argmax(x) would
    # land on some later, unrelated feature. Pick whichever of max/min is
    # further from the pre-pulse baseline instead, so this works for
    # either sign.
    baseline_est = float(np.median(x[:max(1, int(len(x) * 0.1))]))
    if abs(x.max() - baseline_est) >= abs(x.min() - baseline_est):
        rise_idx = int(np.argmax(x))
    else:
        rise_idx = int(np.argmin(x))
    min_idx = int(np.argmin(x))
    back_off_s = 0.06
    off_idx = int(np.searchsorted(t, t[min_idx] - back_off_s))
    off_idx = min(max(off_idx, 0), len(t) - 1)
    t_pulse_off_tick = t[off_idx]
    print(f"Fit window anchored on global-minimum trough at {t[min_idx]*1000:.0f}ms "
          f"(sample {min_idx}); starting fit at {t_pulse_off_tick*1000:.0f}ms "
          f"(sample {off_idx} of {len(t)})")

    # Fit the decay starting shortly after the amp turns off (avoid the
    # amp-on transient itself, which isn't the free-decay portion).
    fit_mask = t > (t_pulse_off_tick + 0.01)
    t_fit = t[fit_mask]
    x_fit = x[fit_mask]

    # Data-driven initial frequency guess (FFT peak on a uniformly-
    # resampled version of the fit window) rather than a hardcoded value
    # -- this project's own prior guesses (11Hz, then 15Hz) both turned
    # out to bias curve_fit toward the wrong answer once real (not
    # host-timestamp-bucketed) resolution revealed the true oscillation
    # was faster than either. Real samples arrive with a few ms of
    # jitter, not perfectly evenly spaced, so interpolate onto a uniform
    # grid first for a clean spectrum.
    if len(t_fit) > 16:
        t_uniform = np.linspace(t_fit[0], t_fit[-1], len(t_fit))
        x_uniform = np.interp(t_uniform, t_fit, x_fit)
        x_uniform = x_uniform - x_uniform.mean()
        dt_uniform = t_uniform[1] - t_uniform[0]
        freqs = np.fft.rfftfreq(len(x_uniform), d=dt_uniform)
        mag = np.abs(np.fft.rfft(x_uniform * np.hanning(len(x_uniform))))
        mag[0] = 0.0  # zero out DC
        freq_guess = float(freqs[np.argmax(mag)])
        if freq_guess < 1.0:
            freq_guess = 15.0
        print(f"FFT-based frequency guess for curve_fit seeding: {freq_guess:.2f}Hz")
    else:
        freq_guess = 15.0
    amp_guess = (x_fit.max() - x_fit.min()) / 2.0
    offset_guess = x_fit[-20:].mean()
    p0 = [amp_guess, freq_guess, 0.05, 0.0, offset_guess, t_pulse_off_tick]
    try:
        popt, pcov = curve_fit(
            lambda tt, amp, freq_hz, zeta, phase, offset: damped_sine(tt, amp, freq_hz, zeta, phase, offset, t_pulse_off_tick),
            t_fit, x_fit, p0=p0[:5], maxfev=20000,
            bounds=([0, 1, 0.001, -np.pi, -1e6], [1e4, 60, 2.0, np.pi, 1e6]))
        amp_fit, freq_fit, zeta_fit, phase_fit, offset_fit = popt
        fit_ok = True
    except Exception as e:
        print(f"Damped-sine fit failed: {e}")
        fit_ok = False

    # Model-free peak/trough-spacing frequency estimate -- added 2026-08-27,
    # folded in from scratch_ringdown_peak_analysis.py (an ad hoc one-off
    # built 2026-08-19 after curve_fit converged on two DIFFERENT wrong
    # answers in a row on the same data before this method settled it).
    # This is now the PRIMARY frequency estimate reported below; curve_fit
    # is kept only for its damping-ratio (zeta) estimate, which peak-
    # spacing alone can't provide. Skips the messier first ~75ms after the
    # trough (still settling from the forced-drive transient), uses a
    # 500ms clean-tail window.
    peak_start_t = t[min_idx] + 0.075
    peak_mask = (t > peak_start_t) & (t < peak_start_t + 0.5)
    tt, xx = t[peak_mask], x[peak_mask]
    peak_idx, _ = find_peaks(xx, prominence=0.5)
    trough_idx, _ = find_peaks(-xx, prominence=0.5)
    peak_freq = None
    if len(peak_idx) + len(trough_idx) >= 2:
        all_spacing = np.concatenate([np.diff(tt[peak_idx]), np.diff(tt[trough_idx])])
        if len(all_spacing) > 0:
            mean_period = all_spacing.mean()
            peak_freq = 1.0 / mean_period
            print(f"\nModel-free peak/trough spacing: {len(peak_idx)} peaks + {len(trough_idx)} troughs, "
                  f"n={len(all_spacing)} spacings, mean={mean_period*1000:.2f}ms -> {peak_freq:.2f}Hz")
    if peak_freq is None:
        print("\nModel-free peak/trough spacing: not enough peaks/troughs found in the clean-tail window.")

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_path = args.out or f"results/fta_ringdown_{args.axis}axis_{ts}.npz"
    save_kwargs = dict(t=t, x=x, x_raw=x_raw, y_raw=y_raw, axis=args.axis, signal_label=signal_label,
                        t_pulse_off_tick=t_pulse_off_tick,
                        t_pulse_on_host_approx=t_pulse_on, t_pulse_off_host_approx=t_pulse_off,
                        base_dac=args.base_dac, offset_dac=args.offset_dac, pulse_ms=args.pulse_ms)
    if fit_ok:
        save_kwargs.update(freq_hz=freq_fit, zeta=zeta_fit, amp_px=amp_fit)
    if peak_freq is not None:
        save_kwargs.update(peak_freq_hz=peak_freq)
    np.savez(out_path, **save_kwargs)
    print(f"Saved raw time series to {out_path}")

    if fit_ok:
        print(f"\ncurve_fit free-decay: freq={freq_fit:.2f}Hz  damping ratio (zeta)={zeta_fit:.3f}  "
              f"amplitude={amp_fit:.2f}px ({amp_fit*MICRONS_PER_PIXEL:.1f}um)  "
              f"[frequency here is secondary -- trust peak_freq above if the two disagree]")
    else:
        print("\ncurve_fit failed -- inspect the plot manually; peak-spacing freq above may still be valid.")

    # --- plot ---
    fig, ax = plt.subplots(figsize=(10, 4.5), dpi=150)
    ax.set_facecolor("white")
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9, length=3)

    ax.plot(t, x, color=BLUE, linewidth=1.2, label=f"measured {signal_label}")
    ax.axvline(t[rise_idx], color=MUTED, linewidth=0.8, linestyle=(0, (1, 2)), label="amp on (approx, peak of rise)")
    ax.axvline(t_pulse_off_tick, color=ORANGE, linewidth=0.8, linestyle=(0, (1, 2)), label="amp off (data-driven)")
    if peak_freq is not None:
        ax.plot(tt[peak_idx], xx[peak_idx], "o", color=ORANGE, markersize=5, zorder=5,
                label=f"peaks/troughs used ({peak_freq:.1f}Hz)")
        ax.plot(tt[trough_idx], xx[trough_idx], "o", color="#2a2a2a", markersize=5, zorder=5)
    if fit_ok:
        t_plot = np.linspace(t_pulse_off_tick, t[-1], 500)
        fit_curve = damped_sine(t_plot, amp_fit, freq_fit, zeta_fit, phase_fit, offset_fit, t_pulse_off_tick)
        ax.plot(t_plot, fit_curve, color="#2a2a2a", linewidth=1.0, linestyle=(0, (4, 2)),
                label=f"curve_fit: {freq_fit:.1f}Hz, zeta={zeta_fit:.3f} (secondary)")

    sec = ax.secondary_yaxis("right", functions=(lambda px: px * MICRONS_PER_PIXEL, lambda v: v / MICRONS_PER_PIXEL))
    sec.tick_params(colors=MUTED, labelsize=9, length=3)
    sec.set_ylabel("µm", fontsize=9, color=MUTED)

    ax.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
    ax.set_ylabel(f"{signal_label} (px)", fontsize=9.5, color=MUTED)
    ax.legend(frameon=False, fontsize=8.5, loc="upper right")

    axis_desc = "dac_y -> cx, primary axis" if args.axis == "x" else "dac_x -> cy, second axis"
    fig.suptitle(f"Free-decay ring-down test ({axis_desc})", fontsize=13, fontweight="bold")
    fig.tight_layout()
    png_path = out_path.rsplit(".", 1)[0] + ".png"
    fig.savefig(png_path, facecolor="white")
    plt.close(fig)
    print(f"Saved plot to {png_path}")


if __name__ == "__main__":
    main()
