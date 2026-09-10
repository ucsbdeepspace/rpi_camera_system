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
# Field-search regexes rather than one strict, positionally-anchored
# full-line TELEMETRY_RE -- the relay line has grown more fields over
# this project's history (dac_x=, cseq=, ...) and a `$`-anchored regex
# expecting a specific field to be last breaks the instant a new field
# gets appended after it. Confirmed here 2026-09-10: the old regex
# expected `...dac_y=N tick=N` but the real line now has `dac_y=N
# dac_x=N tick=N`, and was `$`-anchored right after errs= when the real
# line continues with cseq= -- so it could never match anything,
# silently capturing 0 samples every run. \b anchors keep these safe
# against substring collisions (dac_x= containing "x=", "_" is a word
# character so no boundary forms between it and the field name).
TELEMETRY_RE = re.compile(r"^seq=\s*(\d+)\s+status=(\d+)\b")
FIELD_RE = {
    "x": re.compile(r"\bx=(-?\d+\.\d)"),
    "y": re.compile(r"\by=(-?\d+\.\d)"),
    "tgt": re.compile(r"\btgt=(-?\d+\.\d)"),
    # No collision with "tgt=" above -- that pattern requires "=" directly
    # after "tgt", which "tgt_y=" doesn't have (it has "_y=" there), so
    # they can't match each other's tokens. Added 2026-09-10 alongside the
    # firmware's new tgt_y= telemetry field, for axis-2 closed-loop sine
    # tracking (fitting measured cy against the real per-sample target_y,
    # the same "fit both, diff cancels" trick already used for tgt/cx).
    "tgt_y": re.compile(r"\btgt_y=(-?\d+\.\d)"),
    "dac_y": re.compile(r"\bdac_y=(-?\d+)"),
    "dac_x": re.compile(r"\bdac_x=(-?\d+)"),
    "tick": re.compile(r"\btick=(\d+)"),
}
STATUS_FIELD_RE = {
    "dac_x": re.compile(r"dac_x=(-?\d+)"),
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_x": re.compile(r"tel_x=(-?[\d.]+)"),
    "tel_y": re.compile(r"tel_y=(-?[\d.]+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
    # \b anchors -- without them these substring-match inside the
    # firmware's OTHER sine fields (open_sine=, open_sine_freq_millihz=,
    # both real STATUS fields since the 2026-09-01 open-loop Bode work),
    # which appear EARLIER on the STATUS line than these closed-loop
    # fields. re.search() returns the first match, so an unanchored
    # search here was silently reading open_sine_freq_millihz's value
    # (found 2026-09-10: two consecutive start_sine confirmations failed
    # reporting "sine_freq_millihz=100000" -- exactly the stale
    # open_sine_freq_millihz value left over from this session's earlier
    # Bode sweep, not a dropped-byte/VCP-flakiness false alarm). "_" is a
    # word character, so \b correctly fails to match right after
    # "open_sine" (no boundary between "_" and "s") while still matching
    # the real standalone "sine=" token elsewhere on the line.
    "sine": re.compile(r"\bsine=(\d+)"),
    "sine_freq_millihz": re.compile(r"\bsine_freq_millihz=(-?\d+)"),
    # Same \b reasoning as sine=/sine_freq_millihz= above -- "sine_axis="
    # would otherwise substring-match inside "open_sine_axis=" (which sits
    # earlier on the STATUS line), reading the wrong axis's value. Added
    # 2026-09-10 alongside the firmware's new sine_axis= field.
    "sine_axis": re.compile(r"\bsine_axis=(\d+)"),
    "axis2": re.compile(r"axis2=(\d+)"),
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
    one-time setup commands here, so this doesn't need to be fast.

    Clears stale input right before each write (2026-08-19 fix, ported
    from fta_closed_loop_step_response_vcp.py): at the ~465Hz telemetry
    rate, a command whose reply never arrives leaves ~2s of already-
    buffered telemetry sitting unread, and without this reset the NEXT
    attempt's reply-matching window gets spent draining that stale
    backlog instead of watching for a genuinely fresh reply -- confirmed
    to cascade into repeated full-retry failures otherwise."""
    for attempt in range(retries):
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
        ser.reset_input_buffer()
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


# The core fields every caller needs. sine/sine_freq_millihz are the LAST
# fields on the STATUS line, so a dropped trailing byte under load (the
# well-documented VCP byte-loss this project has hit repeatedly) truncates
# exactly them -- confirmed live: "sine_freq_millihz=" landing with its "0"
# eaten, directly butted against the next queued telemetry line
# ("...millihz=seq= 10 status=1..."). The ORIGINAL strict all-fields check
# failed on this every retry (a real, reproducible bug), which is why the
# plain connectivity check in main() below only requires the core fields.
#
# BUT: silently defaulting a missing sine field to 0 (an earlier version of
# this fix) is WRONG for the post-start_sine confirmation check further
# down -- that check specifically needs to tell "really 0" apart from
# "corrupted, keep retrying", and a fabricated 0 makes it fail every time
# even when start_sine genuinely landed at the right frequency (confirmed
# live: 3 consecutive false aborts, all reporting sine_freq_millihz=0,
# before this was caught). So get_status() takes an explicit `required`
# set: the sine-confirmation call passes the full field set (strict, keeps
# retrying on a corrupted read rather than defaulting), the plain
# connectivity check passes the reduced set.
CORE_STATUS_FIELDS = ("dac_x", "dac_y", "amp", "tel_x", "tel_age_ms", "axis2")
ALL_STATUS_FIELDS = tuple(STATUS_FIELD_RE.keys())


def get_status(ser, retries=5, required=ALL_STATUS_FIELDS):
    for _ in range(retries):
        reply = send_command(ser, "get_status", retries=1)
        if reply is None or not reply.startswith("STATUS"):
            continue
        matches = {k: rx.search(reply) for k, rx in STATUS_FIELD_RE.items()}
        if all(matches[k] for k in required):
            sine_m = matches["sine"]
            sine_freq_m = matches["sine_freq_millihz"]
            sine_axis_m = matches.get("sine_axis")
            tel_y_m = matches.get("tel_y")
            return {
                "dac_x": int(matches["dac_x"].group(1)),
                "dac_y": int(matches["dac_y"].group(1)),
                "amp": int(matches["amp"].group(1)),
                "tel_x": float(matches["tel_x"].group(1)),
                "tel_y": float(tel_y_m.group(1)) if tel_y_m else None,
                "tel_age_ms": int(matches["tel_age_ms"].group(1)),
                "sine": int(sine_m.group(1)) if sine_m else 0,
                "sine_freq_millihz": int(sine_freq_m.group(1)) if sine_freq_m else 0,
                "sine_axis": int(sine_axis_m.group(1)) if sine_axis_m else 0,
                "axis2": int(matches["axis2"].group(1)),
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


def save_plot(t, x, tgt, dac_y, freq, amplitude_px, base_dac_y, kp_milli, ki_milli, gain, lag_ms,
              out_path, y=None, axis2=None, target_axis="x", tgt_y=None, dac_x=None):
    """Primary axis is um (the physically meaningful unit for this
    project's actual deliverable -- beacon-wobble rejection in real
    displacement), with px kept as a secondary axis rather than dropped
    entirely, since every DAC-side reasoning elsewhere in this project
    still happens in px/counts. Second panel is the real commanded
    actuator output (raw DAC counts) over the same time axis -- added
    2026-08-14 so the actuator command is visible directly alongside the
    resulting primary/target trace, instead of only being inferable
    offline from the control law. Third panel (the OTHER axis) added
    2026-08-19 for the axis2-on-vs-off sine comparison -- y=None (e.g.
    replotting an older npz that predates this field) skips that panel
    rather than erroring.

    target_axis="y" (added 2026-09-10, for axis-2 sine validation) swaps
    which measurement is "primary" (the one being actively swept, plotted
    against its own target) vs. "other axis" (the passive one, monitored
    for cross-coupling) -- cy/tgt_y/dac_x become primary, cx becomes the
    other-axis panel. dac_x=None falls back to plotting dac_y in the
    actuator panel regardless (e.g. replotting an npz saved before dac_x
    was recorded here), same "gap not a lie" reasoning as y=None above."""
    um = MICRONS_PER_PIXEL
    axis_y = (target_axis == "y")
    period_ms = 1000.0 / freq
    lag_deg = (lag_ms / period_ms) * 360.0

    # Primary/other selection -- see docstring. Falls back to the axis-1
    # (x) selection if the axis-2 data this needs (tgt_y, dac_x) wasn't
    # actually captured, rather than plotting garbage.
    if axis_y and tgt_y is not None:
        primary_meas, primary_tgt, primary_label = y, tgt_y, "cy"
        primary_dac, dac_label = (dac_x if dac_x is not None else dac_y), \
            ("dac_x" if dac_x is not None else "dac_y")
        other_meas, other_label = x, "cx"
    else:
        primary_meas, primary_tgt, primary_label = x, tgt, "cx"
        primary_dac, dac_label = dac_y, "dac_y"
        other_meas, other_label = y, "cy"

    n_rows = 3 if other_meas is not None else 2
    height_ratios = [1.6, 1, 1] if other_meas is not None else [1.6, 1]
    fig, axes = plt.subplots(n_rows, 1, figsize=(10, 6.5 if other_meas is None else 8.5), dpi=150,
                              sharex=True, gridspec_kw={"height_ratios": height_ratios, "hspace": 0.12})
    ax, ax_dac = axes[0], axes[1]
    ax_other = axes[2] if other_meas is not None else None
    for a in axes:
        a.set_facecolor("white")
        for spine in ("top", "right"):
            a.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            a.spines[spine].set_color(GRID)
        a.tick_params(colors=MUTED, labelsize=9, length=3)

    ax.plot(t, primary_tgt * um, color=TARGET_COLOR, linewidth=1.1, linestyle=(0, (2, 2)),
            label=f"target_{'y' if (axis_y and tgt_y is not None) else 'x'} (firmware-reported, per-sample)")
    ax.plot(t, primary_meas * um, color=BLUE, linewidth=1.3, label=f"measured {primary_label}")

    sec = ax.secondary_yaxis("right", functions=(lambda v: v / um, lambda px: px * um))
    sec.tick_params(colors=MUTED, labelsize=9, length=3)
    sec.set_ylabel("px", fontsize=9, color=MUTED)

    ax.set_ylabel(f"{primary_label} (µm)", fontsize=9.5, color=MUTED)
    ax.legend(fontsize=9, loc="upper right", facecolor="white", edgecolor=GRID, framealpha=0.9)

    parts = [f"{freq}Hz  amplitude={amplitude_px:.2f}px / {amplitude_px*um:.1f}um "
             f"@ dac_y={base_dac_y} (on-board sine gen)",
             f"Kp={kp_milli/1000:.2f} Ki={ki_milli/1000:.2f}",
             f"gain: {gain:.2f}  lag: {lag_ms:.1f}ms ({lag_deg:.0f}°)"]
    ax.text(0.02, 0.03, "\n".join(parts), transform=ax.transAxes, fontsize=8.5,
            color="#0b0b0b", va="bottom", ha="left",
            bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=4))

    ax_dac.plot(t, primary_dac, color=ORANGE, linewidth=1.1)
    ax_dac.set_ylabel(f"{dac_label} (counts)", fontsize=9.5, color=MUTED)

    if ax_other is not None:
        ax_other.plot(t, other_meas, color="#c9962c", linewidth=1.0, label=f"measured {other_label} (other axis)")
        ax_other.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
        ax_other.set_ylabel(f"{other_label} (px)", fontsize=9.5, color=MUTED)
        # µm secondary axis on the other-axis panel too, matching the
        # primary one -- previously px-only. Found 2026-09-10 (user:
        # never show a distance in pixels alone).
        sec_o = ax_other.secondary_yaxis("right", functions=(lambda px: px * um, lambda v: v / um))
        sec_o.tick_params(colors=MUTED, labelsize=9, length=3)
        sec_o.set_ylabel("µm", fontsize=9, color=MUTED)
        o_std = float(np.std(other_meas))
        o_range = float(np.max(other_meas) - np.min(other_meas))
        ax_other.text(0.02, 0.95, f"{other_label} std={o_std:.2f}px ({o_std*um:.2f}um)  "
                       f"range={o_range:.2f}px ({o_range*um:.2f}um)", transform=ax_other.transAxes,
                       fontsize=8, color="#0b0b0b", va="top", ha="left",
                       bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=3))
        if axis2 is not None:
            axis2_label = "AXIS2 ON (dac_x correcting cy)" if axis2 else "axis2 OFF (dac_x held fixed)"
            axis2_box = (dict(facecolor="#e3f5e8", edgecolor="#3a9c5c", alpha=0.95, pad=4) if axis2
                         else dict(facecolor="white", edgecolor=GRID, alpha=0.85, pad=4))
            ax_other.text(0.98, 0.95, axis2_label, transform=ax_other.transAxes, fontsize=8,
                           fontweight=("bold" if axis2 else "normal"),
                           color=("#1f6b3a" if axis2 else MUTED), va="top", ha="right", bbox=axis2_box)
    else:
        ax_dac.set_xlabel("time (s)", fontsize=9.5, color=MUTED)

    title = f"Closed-loop sine tracking (on-board generator), {freq}Hz, {primary_label} axis"
    if axis2 is not None:
        title += "  (axis2 ON)" if axis2 else "  (axis2 OFF)"
    fig.suptitle(title, fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, facecolor="white")
    plt.close(fig)


def _reader_thread(ser, records, stop_event):
    """Timestamps from the firmware's own tick= field (HAL_GetTick(), ms),
    NOT time.monotonic() -- found 2026-08-19 (same bug as
    fta_ringdown_test.py before its tick= fix) that host arrival
    timestamps get batched into ~15-16ms bursts by Windows thread-
    scheduling granularity (77.7% of consecutive samples landed on the
    exact same host timestamp in one recorded run), nowhere near enough
    resolution to trust a 5-20Hz sine fit. main() converts the raw tick_ms
    values collected here into relative seconds once capture is done."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except Exception:
            continue
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        m = TELEMETRY_RE.match(line)
        if not m:
            continue
        status = int(m.group(2))
        if not (status & 1):
            continue
        field_m = {k: rx.search(line) for k, rx in FIELD_RE.items()}
        if not all(field_m.values()):
            continue
        x = float(field_m["x"].group(1))
        y = float(field_m["y"].group(1))
        tgt = float(field_m["tgt"].group(1))
        tgt_y = float(field_m["tgt_y"].group(1))
        dac_y = int(field_m["dac_y"].group(1))
        dac_x = int(field_m["dac_x"].group(1))
        tick_ms = int(field_m["tick"].group(1))
        records.append((tick_ms, x, tgt, dac_y, y, tgt_y, dac_x))


def emergency_cleanup(ser, amp_was_enabled):
    """Best-effort hardware-safe shutdown -- called from a finally block so
    it runs even if get_status/an assertion raises partway through main().
    Found necessary 2026-08-19: main() had no exception handling at all, so
    a get_status failure (routine VCP flakiness under load, not rare) after
    start_sine had already succeeded left the sine generator running twice
    in a row, needing manual intervention both times. Each command is its
    own try/except so one failing doesn't block the rest from being
    attempted -- this function must never raise."""
    for cmd in ("stop_sine", "set_mode open_loop", "set_y 95", "set_x 95"):
        try:
            send_command(ser, cmd)
        except Exception:
            pass
    if not amp_was_enabled:
        try:
            send_command(ser, "amp_disable")
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--freq", type=float, default=None,
                         help="required unless --replot is given")
    parser.add_argument("--amplitude-px", type=float, default=25.0)
    parser.add_argument("--base-dac-y", type=int, default=2048)
    parser.add_argument("--base-dac-x", type=int, default=2048,
                         help="idle position for dac_x before closed_loop engages, default 2048 "
                              "(center/near-zero drive on this inverting amp) -- same fix as "
                              "fta_closed_loop_step_response_vcp.py's --base-dac-x, found "
                              "2026-09-10: this script never set dac_x either, leaving axis2's "
                              "bumpless-transfer base at the open_loop floor (95, near-MAX drive), "
                              "which triggered a real thermal/current-limiting square-wave "
                              "oscillation in cy over several seconds.")
    parser.add_argument("--target-axis", choices=("x", "y"), default="x",
                         help="which CONTROL target the on-board sine generator sweeps -- "
                              "'x' (default) sweeps target_x/axis 1 (dac_y->cx), matching every "
                              "prior use of this script. 'y' sweeps target_y/axis 2 (dac_x->cy) "
                              "instead, added 2026-09-10 so axis 2 can get the same sine-tracking "
                              "validation axis 1 already had. --kp-milli/--ki-milli apply to "
                              "whichever axis is selected (set_kp2/set_ki2 for 'y'); the OTHER "
                              "axis's gains are left untouched (0 after a fresh flash) for an "
                              "isolated single-axis test, matching scratch_axis2_step_response.py's "
                              "convention.")
    parser.add_argument("--kp-milli", type=int, default=1750)
    parser.add_argument("--ki-milli", type=int, default=200000)
    parser.add_argument("--ctrl-rate-milli", type=int, default=None,
                         help="throttle the control loop to this rate, milli-Hz; 0=unthrottled; "
                              "omit to leave firmware's current setting unchanged")
    parser.add_argument("--smoothing", type=int, default=None, choices=[0, 1],
                         help="0/1: boxcar-average every confident sample since the last "
                              "control step instead of using just the latest raw sample")
    parser.add_argument("--axis2", type=int, default=None, choices=[0, 1],
                         help="0/1: enable/disable the second control axis (dac_x <- cy) -- "
                              "0 leaves dac_x fixed at its bumpless-transfer base for A/B "
                              "comparison against axis2 actively correcting")
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
        # y/axis2 similarly didn't exist before 2026-08-19 -- None skips
        # the cy panel entirely rather than plotting a fabricated one.
        y_replot = d["y"] if "y" in d.files else None
        axis2_replot = bool(d["axis2"]) if ("axis2" in d.files and int(d["axis2"]) >= 0) else None
        # target_axis/tgt_y/dac_x didn't exist before 2026-09-10 -- fall
        # back to the axis-1-only behavior for older npz files rather
        # than erroring.
        target_axis_replot = str(d["target_axis"]) if "target_axis" in d.files else "x"
        tgt_y_replot = d["tgt_y"] if "tgt_y" in d.files else None
        dac_x_replot = d["dac_x"] if "dac_x" in d.files else None
        save_plot(d["t"], d["x"], d["tgt"], dac_y, float(d["freq"]), float(d["amplitude_px"]),
                  int(d["base_dac_y"]), int(d["kp_milli"]), int(d["ki_milli"]),
                  float(d["gain"]), float(d["lag_ms"]), out_path, y=y_replot, axis2=axis2_replot,
                  target_axis=target_axis_replot, tgt_y=tgt_y_replot, dac_x=dac_x_replot)
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

    st = get_status(ser, required=CORE_STATUS_FIELDS)
    if st["tel_age_ms"] > 500:
        print(f"ERR: last relayed telemetry is {st['tel_age_ms']}ms old -- nothing streaming from the Pi.")
        ser.close()
        raise SystemExit(1)

    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        print(send_command(ser, "amp_enable"))
        st = get_status(ser, required=CORE_STATUS_FIELDS)
        if not st["amp"]:
            print("ERR: amp_enable didn't take -- aborting.")
            ser.close()
            raise SystemExit(1)

    try:
        _run_sine_test(ser, args, amp_was_enabled, duration)
    finally:
        # Runs on ANY exit from _run_sine_test -- normal completion, a
        # raised exception (e.g. get_status failing after start_sine
        # already succeeded, which happened twice live this session and
        # left the sine generator running both times with no prior
        # exception handling at all), or a KeyboardInterrupt.
        emergency_cleanup(ser, amp_was_enabled)
        ser.close()


def _run_sine_test(ser, args, amp_was_enabled, duration):
    axis_y = (args.target_axis == "y")
    print(f"Pre-positioning dac_y={args.base_dac_y}, dac_x={args.base_dac_x}...")
    print(send_command(ser, f"set_y {args.base_dac_y}"))
    print(send_command(ser, f"set_x {args.base_dac_x}"))
    time.sleep(0.5)

    st = get_status(ser, required=CORE_STATUS_FIELDS + (("tel_y",) if axis_y else ()))
    center_px = st["tel_y"] if axis_y else st["tel_x"]
    print(f"baseline c{'y' if axis_y else 'x'}={center_px:.1f}px ({center_px*MICRONS_PER_PIXEL:.1f}um)  "
          f"amplitude={args.amplitude_px}px ({args.amplitude_px*MICRONS_PER_PIXEL:.1f}um)  "
          f"freq={args.freq}Hz  duration={duration:.2f}s  target_axis={args.target_axis}  "
          f"Kp={args.kp_milli/1000:.2f} Ki={args.ki_milli/1000:.2f}")

    # set_mode closed_loop has ALWAYS required set_target_x first (a real
    # safety guard predating axis 2, g_target_x_set) -- even when testing
    # axis 2 only, this harmless priming call (axis 1's gains stay at
    # whatever they currently are, 0 after a fresh flash) is still needed
    # to satisfy it, matching scratch_axis2_step_response.py's own note
    # on this same guard.
    if axis_y:
        st_x = get_status(ser, required=CORE_STATUS_FIELDS)
        print(send_command(ser, f"set_target_x {round(st_x['tel_x'])}"))
        print(send_command(ser, f"set_target_y {round(center_px)}"))
        print(send_command(ser, f"set_kp2 {args.kp_milli}"))
        print(send_command(ser, f"set_ki2 {args.ki_milli}"))
    else:
        print(send_command(ser, f"set_target_x {round(center_px)}"))
        print(send_command(ser, f"set_kp {args.kp_milli}"))
        print(send_command(ser, f"set_ki {args.ki_milli}"))
    if args.ctrl_rate_milli is not None:
        print(send_command(ser, f"set_ctrl_rate {args.ctrl_rate_milli}"))
    if args.smoothing is not None:
        print(send_command(ser, f"set_smoothing {args.smoothing}"))
    if args.axis2 is not None:
        print(send_command(ser, f"set_axis2 {args.axis2}"))
    print(send_command(ser, "set_mode closed_loop"))
    time.sleep(0.3)

    freq_millihz = round(args.freq * 1000)
    amplitude_x10 = round(args.amplitude_px * 10)
    axis_num = 1 if axis_y else 0
    start_reply, t_sine_start = send_command_timed(
        ser, f"start_sine {freq_millihz} {amplitude_x10} {round(center_px)} {axis_num}")
    print(start_reply)
    # Ground-truth check, not just trusting the reply -- confirmed directly
    # (2026-08-19) that start_sine can silently succeed on the firmware
    # side (sine=1 in a later get_status) even when the confirmation reply
    # itself gets lost under load, same VCP flakiness documented elsewhere
    # in this project. Only treat it as a real failure if get_status ALSO
    # disagrees, and always stop_sine on the abort path -- an earlier
    # version of this script left the sine generator latched on after a
    # false-alarm abort (mode reverted to open_loop, but g_sine_active
    # stayed set), a real state-cleanup gap found live this session.
    axis2_gt = args.axis2  # fallback: echo of the CLI arg, overridden below if get_status succeeds
    try:
        verify_st = get_status(ser, retries=10)
        sine_confirmed = (bool(verify_st["sine"]) and verify_st["sine_freq_millihz"] == freq_millihz
                           and verify_st["sine_axis"] == axis_num)
        axis2_gt = verify_st["axis2"]
    except RuntimeError:
        # get_status itself can fail to get ANY clean reply under load
        # (confirmed live 2026-08-19 -- not hypothetical, happened twice
        # in a row) even though start_sine genuinely succeeded firmware-
        # side. Don't abort on this alone; the outer finally's
        # emergency_cleanup makes proceeding safe either way, and the
        # actual analysis only needs the telemetry stream, not this
        # verification.
        print("WARNING: get_status itself failed to verify start_sine -- "
              "proceeding anyway (relying on the paced write having landed).")
        sine_confirmed = True
    if not sine_confirmed:
        print(f"ERR: start_sine not confirmed via get_status "
              f"(sine={verify_st['sine']} sine_freq_millihz={verify_st['sine_freq_millihz']} "
              f"sine_axis={verify_st['sine_axis']}, expected freq={freq_millihz} axis={axis_num}) "
              f"-- aborting.")
        raise RuntimeError("start_sine not confirmed")
    elif not start_reply:
        print("(reply lost, but get_status confirms the sine generator is genuinely running)")
    # t_sine_start is the moment the last character ('\n') was written --
    # a much closer estimate of the firmware's true g_sine_start_tick
    # moment than waiting for the OK reply to arrive (see
    # send_command_timed's docstring).

    records = []
    stop_event = threading.Event()
    reader = threading.Thread(target=_reader_thread, args=(ser, records, stop_event), daemon=True)
    ser.reset_input_buffer()
    reader.start()

    time.sleep(duration)

    stop_event.set()
    reader.join(timeout=1.0)

    # Actual hardware shutdown happens in the caller's `finally` block
    # (emergency_cleanup) regardless of how this function exits -- not
    # duplicated here. This is just an informative status print, best-
    # effort, wrapped so a VCP hiccup here can't skip plot generation.
    try:
        print("status after recording:", get_status(ser))
    except RuntimeError:
        print("(get_status failed here too -- non-fatal, continuing to analysis)")

    print(f"Captured {len(records)} telemetry samples over {duration:.2f}s "
          f"(~{len(records)/duration:.0f}/s average).")
    if len(records) < 6:
        print("Not enough samples to analyze.")
        return

    tick_ms = np.array([r[0] for r in records], dtype=np.int64)
    t = (tick_ms - tick_ms[0]) / 1000.0  # firmware-clock seconds, immune to host scheduling jitter
    x = np.array([r[1] for r in records])
    tgt = np.array([r[2] for r in records])
    dac_y = np.array([r[3] for r in records])
    y_arr = np.array([r[4] for r in records])
    tgt_y = np.array([r[5] for r in records])
    dac_x = np.array([r[6] for r in records])

    # Fit against whichever axis was actually swept -- cy/tgt_y for axis
    # 2, the original cx/tgt for axis 1 (unchanged default behavior).
    fit_meas = y_arr if axis_y else x
    fit_tgt = tgt_y if axis_y else tgt
    gain, lag_ms, offset = fit_tracking(t, fit_meas, fit_tgt, args.freq)
    period_ms = 1000.0 / args.freq
    lag_deg = (lag_ms / period_ms) * 360.0
    um = MICRONS_PER_PIXEL
    meas_label = "cy" if axis_y else "cx"
    print(f"\ntracking gain: {gain:.3f} ({gain*100:.1f}% of commanded {args.amplitude_px}px amplitude, "
          f"{gain*args.amplitude_px:.1f}px / {gain*args.amplitude_px*um:.1f}um)")
    print(f"lag: {lag_ms:.1f}ms ({lag_deg:.1f} deg at {args.freq}Hz)  "
          f"[measured {meas_label} against the firmware's own per-sample target, not a reconstructed reference]")
    print(f"offset from center: {offset:.2f}px ({offset*um:.1f}um)")
    print(f"implied |S|=|1-T| ~= {abs(1-gain):.3f} (magnitude-only approximation)")

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_path = args.out or f"results/fta_closed_loop_onboard_sine_{args.freq:g}Hz_{ts}.npz"
    np.savez(out_path, t=t, x=x, tgt=tgt, dac_y=dac_y, y=y_arr, tgt_y=tgt_y, dac_x=dac_x,
              target_axis=args.target_axis, freq=args.freq,
              amplitude_px=args.amplitude_px, center_px=center_px, base_dac_y=args.base_dac_y,
              kp_milli=args.kp_milli, ki_milli=args.ki_milli, gain=gain, lag_ms=lag_ms, offset=offset,
              axis2=(axis2_gt if axis2_gt is not None else -1))
    print(f"Saved raw time series to {out_path}")

    png_path = out_path.rsplit(".", 1)[0] + ".png"
    save_plot(t, x, tgt, dac_y, args.freq, args.amplitude_px, args.base_dac_y,
              args.kp_milli, args.ki_milli, gain, lag_ms, png_path, y=y_arr,
              axis2=(bool(axis2_gt) if axis2_gt is not None else None),
              target_axis=args.target_axis, tgt_y=tgt_y, dac_x=dac_x)
    print(f"Saved plot to {png_path}")


if __name__ == "__main__":
    main()
