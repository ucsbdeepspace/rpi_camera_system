#!/usr/bin/env python3
"""
Measures the real closed-loop delay (DAC command -> camera-based telemetry
first reflecting the beam having moved) using the firmware's new
`pulse_step DELTA` command and `pulse_tick=` get_status field -- both ends
of the measurement are timestamped by the FIRMWARE's own HAL_GetTick(),
never a host clock, avoiding every host-timing trap this project has hit
this session (the on-board sine generator's original negative-lag bug,
and two separate host-timestamp-bucketing bugs in other scripts).

One pulse per trial, matching this project's established "don't read VCP
replies while the reader thread owns ser.readline()" discipline: the
pulse itself is a fire-and-forget paced write during recording; pulse_tick
is only read back via get_status AFTER the reader thread has stopped.

Usage:
  python3 fta_loop_delay_test.py [--delta N] [--trials N] [--base-dac-y N]
      [--port PORT] [--out PATH]
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

FTA_BAUD = 460800
MICRONS_PER_PIXEL = 3.0

REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+tick=(\d+)\s+pkts=(\d+)\s+errs=(\d+)$")
STATUS_FIELD_RE = {
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_x": re.compile(r"tel_x=(-?[\d.]+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
    "pulse_tick": re.compile(r"pulse_tick=(\d+)"),
}

BLUE = "#2a78d6"
MUTED = "#898781"
GRID = "#e1e0d9"
PULSE_COLOR = "#c9962c"


def find_fta_port():
    from serial.tools import list_ports
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    return candidates[0].device if candidates else None


def send_command(ser, cmd, char_delay=0.02, reply_timeout=2.0, retries=5):
    """Paced write with a stale-input reset before each attempt -- see
    fta_closed_loop_step_response_vcp.py's send_command docstring for why
    the reset matters (a timed-out command otherwise leaves ~2s of
    telemetry backlog that starves the NEXT command's own reply window)."""
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


def get_status(ser, retries=5):
    for _ in range(retries):
        reply = send_command(ser, "get_status", retries=1)
        if reply is None or not reply.startswith("STATUS"):
            continue
        matches = {k: rx.search(reply) for k, rx in STATUS_FIELD_RE.items()}
        if all(matches.values()):
            return {
                "dac_y": int(matches["dac_y"].group(1)),
                "amp": int(matches["amp"].group(1)),
                "tel_x": float(matches["tel_x"].group(1)),
                "tel_age_ms": int(matches["tel_age_ms"].group(1)),
                "pulse_tick": int(matches["pulse_tick"].group(1)),
            }
    raise RuntimeError("No parseable get_status reply after several attempts.")


def _reader_thread(ser, records, stop_event):
    """Sole reader of the serial port for the whole recording window.
    Timestamps from the firmware's tick= field only -- no host clock
    involved anywhere in this measurement."""
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
        tick_ms = int(m.group(7))
        records.append((tick_ms, x))


def run_trial(ser, base_dac_y, delta, pre_s, post_s):
    """One pulse, one measurement. Returns a dict with the raw arrays and
    the computed delay, or None if the trial couldn't be analyzed."""
    print(send_command(ser, f"set_y {base_dac_y}"))
    time.sleep(0.4)  # let it settle before this trial's baseline window

    records = []
    stop_event = threading.Event()
    reader = threading.Thread(target=_reader_thread, args=(ser, records, stop_event), daemon=True)
    ser.reset_input_buffer()
    reader.start()

    time.sleep(pre_s)
    # Fire-and-forget, paced -- does NOT read a reply here (the reader
    # thread owns ser.readline() for the whole window; two threads
    # reading the same Serial object would race and corrupt the stream).
    for ch in f"pulse_step {delta}\n":
        ser.write(ch.encode("ascii"))
        time.sleep(0.02)
    time.sleep(post_s)

    stop_event.set()
    reader.join(timeout=1.0)

    st = get_status(ser)  # safe now -- reader thread has stopped
    pulse_tick = st["pulse_tick"]

    if len(records) < 10:
        print(f"  only {len(records)} samples captured -- skipping trial")
        return None

    tick_ms = np.array([r[0] for r in records], dtype=np.float64)
    x = np.array([r[1] for r in records])
    t = (tick_ms - pulse_tick) / 1000.0  # seconds relative to the pulse itself

    pre_mask = t < 0
    if pre_mask.sum() < 5:
        print("  not enough pre-pulse samples -- skipping trial")
        return None
    baseline = x[pre_mask].mean()
    noise_std = x[pre_mask].std()
    threshold = max(1.0, 3.0 * noise_std)

    post_mask = t >= 0
    post_t = t[post_mask]
    post_dev = np.abs(x[post_mask] - baseline)
    onset_idx = np.argmax(post_dev > threshold) if np.any(post_dev > threshold) else -1
    if onset_idx == -1:
        print(f"  never exceeded threshold ({threshold:.2f}px) -- skipping trial")
        return None
    delay_s = post_t[onset_idx]

    return {
        "t": t, "x": x, "baseline": baseline, "noise_std": noise_std,
        "threshold": threshold, "delay_s": delay_s, "delta": delta,
        "pulse_tick": pulse_tick,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--delta", type=int, default=200, help="DAC-count step per pulse (default 200)")
    parser.add_argument("--trials", type=int, default=6)
    parser.add_argument("--base-dac-y", type=int, default=2048)
    parser.add_argument("--pre-s", type=float, default=0.15)
    parser.add_argument("--post-s", type=float, default=0.35)
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
        print(f"ERR: last relayed telemetry is {st['tel_age_ms']}ms old -- "
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

    trials = []
    for i in range(args.trials):
        delta = args.delta if i % 2 == 0 else -args.delta
        print(f"trial {i+1}/{args.trials}: delta={delta:+d} counts")
        result = run_trial(ser, args.base_dac_y, delta, args.pre_s, args.post_s)
        if result is not None:
            print(f"  delay: {result['delay_s']*1000:.1f}ms  "
                  f"(baseline={result['baseline']:.1f}px, threshold={result['threshold']:.2f}px)")
            trials.append(result)

    print(send_command(ser, "set_y 95"))
    if not amp_was_enabled:
        print(send_command(ser, "amp_disable"))
    print("final:", get_status(ser))
    ser.close()

    if not trials:
        print("No usable trials -- nothing to report.")
        return

    delays_ms = np.array([r["delay_s"] * 1000 for r in trials])
    print(f"\n{len(trials)}/{args.trials} usable trials")
    print(f"delay: mean={delays_ms.mean():.1f}ms  median={np.median(delays_ms):.1f}ms  "
          f"std={delays_ms.std():.1f}ms  min={delays_ms.min():.1f}ms  max={delays_ms.max():.1f}ms")

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_path = args.out or f"results/fta_loop_delay_{ts}.npz"
    np.savez(out_path,
             delays_ms=delays_ms,
             trial_t=np.array([r["t"] for r in trials], dtype=object),
             trial_x=np.array([r["x"] for r in trials], dtype=object),
             trial_baseline=np.array([r["baseline"] for r in trials]),
             trial_threshold=np.array([r["threshold"] for r in trials]),
             delta=args.delta, base_dac_y=args.base_dac_y)
    print(f"Saved raw data to {out_path}")

    # --- plot: one panel per trial, delay marked ---
    n = len(trials)
    fig, axes = plt.subplots(n, 1, figsize=(8, 2.2 * n), dpi=150, sharex=False)
    if n == 1:
        axes = [axes]
    for ax, r in zip(axes, trials):
        ax.set_facecolor("white")
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            ax.spines[spine].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=8, length=3)
        t_ms = r["t"] * 1000
        ax.plot(t_ms, r["x"], color=BLUE, linewidth=1.2)
        ax.axhline(r["baseline"], color=MUTED, linewidth=0.8, linestyle=(0, (2, 2)))
        ax.axvline(0, color=PULSE_COLOR, linewidth=1.2, label="pulse_step applied (firmware tick)")
        ax.axvline(r["delay_s"] * 1000, color="#8a1f1f", linewidth=1.2, linestyle=(0, (3, 2)),
                   label=f"onset (+{r['delay_s']*1000:.1f}ms)")
        ax.set_ylabel("cx (px)", fontsize=8.5, color=MUTED)
        ax.legend(frameon=False, fontsize=7.5, loc="upper right" if r["delta"] > 0 else "lower right")
        ax.set_title(f"delta={r['delta']:+d} counts, delay={r['delay_s']*1000:.1f}ms",
                     fontsize=9, color="#0b0b0b")
    axes[-1].set_xlabel("time relative to pulse_step (ms, firmware clock)", fontsize=9, color=MUTED)
    fig.suptitle(f"Closed-loop delay measurement -- mean={delays_ms.mean():.1f}ms "
                 f"median={np.median(delays_ms):.1f}ms (n={n})",
                 fontsize=12, fontweight="bold")
    fig.tight_layout()
    png_path = out_path.rsplit(".", 1)[0] + ".png"
    fig.savefig(png_path, facecolor="white")
    plt.close(fig)
    print(f"Saved plot to {png_path}")


if __name__ == "__main__":
    main()
