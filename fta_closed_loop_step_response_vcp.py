#!/usr/bin/env python3
"""
Characterizes the CLOSED-LOOP step response of the dac_y -> cx PID pathway
implemented in camera_centroid_receiver (MODE_CLOSED_LOOP, added
2026-08-13) -- rise time, overshoot, settling time of the measured pixel
error following a target_x step, the same three metrics
fta_step_response_test_vcp.py already computes for the OPEN-LOOP plant.
That script characterizes the actuator itself; this one characterizes the
actual control loop that will run in real operation.

Only the dac_y->cx pathway exists in firmware right now (the other axis
was never implemented -- see CLAUDE.md's "Architecture DECISION v2" and
the 2026-08-12 axis-choice section), so there's no --axis option the way
the open-loop script has one.

Bench findings this session (2026-08-13, see CLAUDE.md) this script's
defaults are built on:
  - dac_y=2048 is the cleanest, most linear region found on this rig this
    session (minor-loop hysteresis gap ~0.04px, consistent slope both
    directions) -- default --base-dac-y.
  - Kp=1.75 counts/px, Ki=200 counts/(px*s) give a clean, single-
    transition step response in ~141ms (rise and settling both), used as
    this script's own first real result. An EARLIER interactive escalation
    search (crude ~3-4Hz terminal polling, not this script) claimed no
    overshoot existed up to Ki=300 -- that claim turned out to be a
    measurement-resolution artifact, not a real absence of overshoot: this
    script's own high-rate (~135Hz) logging found real, visible ringing at
    Ki=400 (15% overshoot), and the interactive search's polling rate was
    too slow to resolve oscillation on that timescale. Raising Kp instead
    of Ki was tried and made things WORSE (more ringing, no faster
    settling) -- Ki was the actual bottleneck the whole time, not Kp.
    See CLAUDE.md for the full comparison table.
  - A single ser.write() burst of a whole VCP command line reliably loses
    bytes at the Pi's current high telemetry rate (~150-200Hz) -- every
    SETUP command here is sent via send_command(), which paces the write
    at ~20ms/char and confirms the reply, exactly the workaround found
    this session. The STEP command itself (the one whose exact timing
    this script measures) is sent as a single fast burst instead, to
    preserve precise step-onset timing the way fta_step_response_test_vcp.py
    does -- but verified via a paced get_status AFTER recording stops, so
    a silently-dropped step (which would otherwise just look like "the
    actuator didn't respond") is caught and reported instead of trusted.

Usage:
  python3 fta_closed_loop_step_response_vcp.py [--base-dac-y N]
      [--step-px N] [--kp-milli N] [--ki-milli N] [--pre-s SEC]
      [--post-s SEC] [--settle-tol-px PX] [--port PORT] [--out PATH]

    --base-dac-y N     open-loop pre-position before engaging the loop,
                        default 2048 (see above).
    --step-px N         target_x step size in pixels, default -25
                        (matches this session's tested convention; sign
                        chooses direction, magnitude should stay in the
                        small-step regime this project has established
                        elsewhere -- large steps haven't been
                        characterized for this control pathway).
    --kp-milli N        Kp * 1000 (firmware's own units), default 1750.
    --ki-milli N        Ki * 1000, default 200000.
    --pre-s SEC         seconds recorded BEFORE the step, holding at
                        target=baseline under closed-loop control (not
                        open-loop -- this captures the loop's own hold
                        noise/behavior, not just plant baseline), default
                        0.5.
    --post-s SEC        seconds recorded AFTER the step, default 3.0.
    --settle-tol-px PX  settling-band tolerance, default 2.0px (6.0um).
    --port PORT         Nucleo VCP serial port, default auto-detect.
    --out PATH          raw (t, x, y) npz path -- default
                        results/fta_closed_loop_step_response_vcp_<UTC
                        timestamp>.npz. A PNG plot is saved alongside it
                        (same path, .png extension).

Requires the Pi to already be streaming telemetry, same precondition as
fta_step_response_test_vcp.py -- checked via get_status's tel_age_ms
before starting.
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

FTA_BAUD = 115200  # camera_centroid_receiver's USART2 rate.
MICRONS_PER_PIXEL = 3.0  # OV9281 pixel pitch, same constant used throughout this project.

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
    "target_x_set": re.compile(r"target_x_set=(\d+)"),
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
    """Paces the write at ~20ms/char rather than one ser.write() burst --
    found necessary this session (2026-08-13, see CLAUDE.md and this
    module's docstring): a burst write of a whole command line reliably
    loses bytes at the Pi's current high telemetry rate. Returns the
    first OK/ERR/STATUS/WARN reply line seen, or None on timeout."""
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
    """Paced get_status with field parsing, retried on a corrupted/missing
    reply rather than trusting the first attempt."""
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
                "target_x_set": int(matches["target_x_set"].group(1)),
            }
    raise RuntimeError("No parseable get_status reply after several attempts -- check the serial link/firmware.")


def analyze_step(t, primary, t_step, settle_tol_px):
    """Based on fta_step_response_test_vcp.py's analyze_step (duplicated
    per this project's established convention -- see that function's own
    docstring), but with a real bug fixed: that version's first_crossing
    switched from `frac >= target` to `frac <= target` when delta<0, on
    the assumption a falling signal needs the comparison flipped. It
    doesn't -- frac = (v - baseline) / delta is already sign-normalized
    by the division, so it goes 0 -> 1 as the signal moves from baseline
    to final REGARDLESS of whether the raw value is rising or falling
    (verified against this script's own real step data, 2026-08-13: a
    -25px step's frac trace climbs 0->1 monotonically like any other,
    confirmed with a hand-checked synthetic example too). The old
    conditional made first_crossing(0.90) trigger on the very first
    post-step sample for any falling step (any frac below 0.90, including
    near-zero noise, satisfied `frac <= 0.90`), landing t90 before t10 and
    silently reporting "rise time: could not be determined" every single
    time -- not a fluke, this would have hit every closed-loop test this
    session, all of which stepped in the negative direction. Only
    first_crossing itself needed the fix; everything else in this
    function (baseline/final/overshoot/settling) was unaffected since
    those don't depend on frac's crossing direction."""
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
                "note": "no net change detected"}

    post_t = t[post_mask] - t_step
    post_v = primary[post_mask]
    frac = (post_v - baseline) / delta

    def first_crossing(target_frac):
        idx = np.where(frac >= target_frac)[0]
        return post_t[idx[0]] if len(idx) else None

    t10 = first_crossing(0.10)
    t90 = first_crossing(0.90)
    rise_time = (t90 - t10) if (t10 is not None and t90 is not None and t90 >= t10) else None

    # Same sign-normalization argument as first_crossing above: frac already
    # reads 0->1(+) regardless of delta's sign, so the peak (overshoot) is
    # always just its max -- the old delta<0 branch (`-np.min(frac)`) measured
    # something unrelated to overshoot and could silently under-report it.
    max_frac = float(np.max(frac))
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
    """Sole reader of the serial port for the whole recording window --
    same discipline as fta_step_response_test_vcp.py: the main thread only
    writes (the single burst step command) while this thread is running,
    never reads, so lines never get split across two concurrent readers."""
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


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base-dac-y", type=int, default=2048)
    parser.add_argument("--step-px", type=float, default=-25.0)
    parser.add_argument("--kp-milli", type=int, default=1750)
    parser.add_argument("--ki-milli", type=int, default=200000)
    parser.add_argument("--pre-s", type=float, default=0.5)
    parser.add_argument("--post-s", type=float, default=3.0)
    parser.add_argument("--settle-tol-px", type=float, default=2.0)
    parser.add_argument("--port", default=None)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

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
              "nothing appears to be streaming from the Pi. Start "
              "camera_view_tool.py or beam_position_streamer.py there first.")
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
    baseline_cx = st["tel_x"]
    target_from = round(baseline_cx)
    target_to = round(baseline_cx + args.step_px)
    print(f"baseline cx={baseline_cx:.1f}  target_from={target_from}  "
          f"target_to={target_to} (step {args.step_px:+.1f}px)  "
          f"Kp_milli={args.kp_milli} Ki_milli={args.ki_milli}")

    print(send_command(ser, f"set_target_x {target_from}"))
    print(send_command(ser, f"set_kp {args.kp_milli}"))
    print(send_command(ser, f"set_ki {args.ki_milli}"))
    print(send_command(ser, "set_mode closed_loop"))
    time.sleep(0.3)  # let it settle at the zero-error hold before recording

    records = []
    stop_event = threading.Event()
    t0 = time.monotonic()
    reader = threading.Thread(target=_reader_thread, args=(ser, t0, records, stop_event), daemon=True)
    reader.start()

    time.sleep(args.pre_s)
    # The measured step. A single burst write here was tried first and
    # DID lose bytes on the very first live run of this script (caught by
    # the post-recording target_x verification below, not just a
    # theoretical risk) -- paced instead, matching send_command's write
    # side. Deliberately does NOT read a reply here: the reader thread
    # owns ser.readline() for the whole recording window, and having two
    # threads read the same Serial object concurrently would race and
    # could corrupt the telemetry stream itself. t_step is stamped after
    # the last character is sent, not after a confirmed reply -- still
    # far more precise than risking the whole step going missing.
    for ch in f"set_target_x {target_to}\n":
        ser.write(ch.encode("ascii"))
        time.sleep(0.02)
    t_step = time.monotonic() - t0
    time.sleep(args.post_s)

    stop_event.set()
    reader.join(timeout=1.0)

    # Verify the step command actually landed -- a burst write CAN still
    # lose bytes (that's exactly why setup commands above are paced); this
    # confirms the recorded data reflects a real step, not a dropped one.
    st = get_status(ser)
    if round(st["target_x"]) != target_to:
        print(f"WARNING: post-recording get_status reports target_x={st['target_x']}, "
              f"not the intended {target_to} -- the step command was likely corrupted "
              "in flight. Treat this run's data as suspect; rerun.")

    print(send_command(ser, "set_mode open_loop"))
    print(send_command(ser, "set_y 95"))
    if not amp_was_enabled:
        print(send_command(ser, "amp_disable"))
    print("final:", get_status(ser))
    ser.close()

    if len(records) < 6:
        print(f"Only {len(records)} usable telemetry samples -- not enough to analyze.")
        return

    t = np.array([r[0] for r in records])
    x = np.array([r[1] for r in records])
    y = np.array([r[2] for r in records])
    span = t[-1] - t[0]
    print(f"Captured {len(records)} telemetry samples ({span:.3f}s span, "
          f"~{len(records) / span if span > 0 else 0:.0f}/s average).")

    metrics = analyze_step(t, x, t_step, args.settle_tol_px)
    um = MICRONS_PER_PIXEL
    if metrics is None:
        print("Not enough samples before/after the step to compute metrics.")
    else:
        print(f"baseline={metrics['baseline']:.2f}px ({metrics['baseline']*um:.1f}um)  "
              f"final={metrics['final']:.2f}px ({metrics['final']*um:.1f}um)  "
              f"delta={metrics['delta']:.2f}px ({metrics['delta']*um:.1f}um)")
        if metrics["rise_time_s"] is not None:
            print(f"rise time (10%-90%): {metrics['rise_time_s'] * 1000:.1f}ms")
        else:
            print("rise time: could not be determined")
        if metrics["overshoot_pct"] is not None:
            print(f"overshoot: {metrics['overshoot_pct']:.1f}%")
        if metrics["settling_time_s"] is not None:
            print(f"settling time (within {args.settle_tol_px}px / "
                  f"{args.settle_tol_px*um:.1f}um): {metrics['settling_time_s'] * 1000:.1f}ms")
        if metrics["note"]:
            print(f"NOTE: {metrics['note']}")

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_path = args.out or f"results/fta_closed_loop_step_response_vcp_{ts}.npz"
    np.savez(out_path, t=t, x=x, y=y, t_step=t_step,
              base_dac_y=args.base_dac_y, step_px=args.step_px,
              target_from=target_from, target_to=target_to,
              kp_milli=args.kp_milli, ki_milli=args.ki_milli)
    print(f"Saved raw time series to {out_path}")

    # --- plot ---
    fig, ax = plt.subplots(figsize=(9, 4.5), dpi=150)
    ax.set_facecolor("white")
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9, length=3)

    ax.axhline(target_from, color=TARGET_COLOR, linewidth=1.0, linestyle=(0, (2, 2)), alpha=0.7)
    ax.plot([t_step, t[-1]], [target_to, target_to], color=TARGET_COLOR, linewidth=1.2,
            linestyle=(0, (2, 2)), label="target_x")
    ax.plot(t, x, color=BLUE, linewidth=1.4, label="measured cx")
    ax.axvline(t_step, color=MUTED, linewidth=0.8, linestyle=(0, (1, 2)))

    sec = ax.secondary_yaxis("right", functions=(lambda px: px * um, lambda v: v / um))
    sec.tick_params(colors=MUTED, labelsize=9, length=3)
    sec.set_ylabel("µm", fontsize=9, color=MUTED)

    ax.set_xlabel("time (s)", fontsize=9.5, color=MUTED)
    ax.set_ylabel("cx (px)", fontsize=9.5, color=MUTED)
    ax.legend(frameon=False, fontsize=9, loc="upper right")

    if metrics is not None:
        parts = [f"step: {args.step_px:+.1f}px @ dac_y={args.base_dac_y}",
                 f"Kp={args.kp_milli/1000:.2f} Ki={args.ki_milli/1000:.2f}"]
        if metrics["rise_time_s"] is not None:
            parts.append(f"rise: {metrics['rise_time_s']*1000:.0f}ms")
        if metrics["overshoot_pct"] is not None:
            parts.append(f"overshoot: {metrics['overshoot_pct']:.1f}%")
        if metrics["settling_time_s"] is not None:
            parts.append(f"settling ({args.settle_tol_px}px): {metrics['settling_time_s']*1000:.0f}ms")
        else:
            parts.append("settling: not reached in window")
        ax.text(0.02, 0.03, "\n".join(parts), transform=ax.transAxes, fontsize=8.5,
                color="#0b0b0b", va="bottom", ha="left",
                bbox=dict(facecolor="white", edgecolor=GRID, alpha=0.9, pad=4))

    fig.suptitle("Closed-loop step response, dac_y → cx", fontsize=13, fontweight="bold")
    fig.tight_layout()
    png_path = out_path.rsplit(".", 1)[0] + ".png"
    fig.savefig(png_path, facecolor="white")
    plt.close(fig)
    print(f"Saved plot to {png_path}")


if __name__ == "__main__":
    main()
