#!/usr/bin/env python3
"""
Measures REAL round-trip latency (and fire-and-forget burst throughput) to
the FTA Controller Nucleo over its USB-serial link (460800 baud), to replace
the physics-based *estimate* discussed in CLAUDE.md's "FTA position
calibration" section with actual numbers before committing to serial as the
real-time actuator command channel (vs. the I2C link nucleo_i2c_sender.py
uses elsewhere in this project -- a different physical connection).

Three independent, standalone measurements (pick with --mode):

  ping (default) -- round trip via `get_status`, a read-only query (no
    actuator movement, safe to run thousands of times). The cleanest
    "protocol + USB overhead" measurement, decoupled from any
    actuator-specific behavior.

  setpos -- round trip via `set_x <current x_center>` (reads the current
    position first via one get_status, then repeatedly re-sends THAT SAME
    value -- no net movement, but exercises the actual command shape and
    per-command acknowledgement line ("x_center set to N") real-time
    control would use). Represents the wait-for-ack pattern
    FTA_GUI_PID.py's existing host driver uses.

  burst -- fire-and-forget throughput AT MAX SPEED: sends --burst-n `set_x
    <current x_center>` commands back-to-back with NO waiting between them
    (true fire-and-forget, no ack read, no pacing), then diffs the
    firmware's own `cmdq_stats` drop counter (a 64-deep command queue, see
    main.c's cmd_q_dropped/CMD_Q_SIZE) before vs. after to check whether
    any were lost. Proves "too fast" but doesn't find the safe ceiling --
    see `sweep` for that.

  sweep -- paced fire-and-forget at a list of candidate fixed rates
    (--sweep-rates, Hz, default spans 100-2500), --sweep-n commands at
    each rate, checking cmdq_stats drops (before/after diff, same as
    burst) at every rate to find the highest one with zero drops. Reports
    BOTH the requested and actually-achieved send rate per row -- at high
    requested rates Python's own loop/timing overhead can itself become
    the limit before the firmware does, so a requested rate whose achieved
    rate falls well short of it isn't really testing what it claims.

For ping/setpos, reports min/mean/median/p95/p99/max round-trip time in
milliseconds over --trials repeats (default 300).

All modes are non-destructive by design: get_status is read-only, and
set_x is always re-sent with its OWN currently-read value, so the FTA
never actually moves -- safe to run against real hardware repeatedly.

Usage:
  python3 fta_serial_latency_test.py [--mode ping|setpos|burst|sweep]
      [--trials N] [--burst-n N] [--sweep-rates R1,R2,...] [--sweep-n N]
      [--port PORT] [--baud BAUD]

Requires the Nucleo's USB connected directly to this Pi (not the laptop).
Not yet run against real hardware -- written from the firmware source in
ucsbdeepspace/7-element-array (lock_in_2), same as fta_calibration.py.
"""
import argparse
import re
import statistics
import time

STATUS_RE = re.compile(r"^status:(-?\d+),(-?\d+),(-?\d+),")
SET_X_ACK_RE = re.compile(r"^x_center set to (-?\d+)\s*$")
CMDQ_STATS_RE = re.compile(r"^cmdq depth=(\d+) dropped=(\d+)\s*$")


def find_fta_port():
    """Auto-detect the Nucleo's USB-serial port by USB description -- same
    tags nucleo_serial_monitor.py/fta_calibration.py use for the ST-Link
    VCP. Duplicated here rather than imported from fta_calibration.py to
    avoid pulling in its cv2/picamera2 imports for a pure serial test."""
    from serial.tools import list_ports
    candidates = [
        p for p in list_ports.comports()
        if any(tag in (p.description or "") for tag in ("STLink", "ST-Link", "STMicroelectronics"))
    ]
    if not candidates:
        return None
    return candidates[0].device


def _round_trip(ser, command, reply_re):
    """Write one command, time until a line matching reply_re arrives (or
    the overall 1s budget below runs out). Returns elapsed seconds, or None
    on timeout/no match -- a dropped/garbled reply is skipped rather than
    silently corrupting the latency stats as a bogus fast/slow outlier."""
    t0 = time.monotonic()
    ser.write((command + "\n").encode("ascii"))
    while time.monotonic() - t0 < 1.0:
        raw = ser.readline()
        if not raw:
            continue  # ser's own read timeout elapsed with no data at all
        text = raw.decode(errors="replace").strip()
        if reply_re.match(text):
            return time.monotonic() - t0
        # else: some other line -- keep reading within this same window
    return None


def get_current_x(ser):
    ser.reset_input_buffer()
    ser.write(b"get_status\n")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        m = STATUS_RE.match(raw.decode(errors="replace").strip())
        if m:
            return int(m.group(2))
    raise RuntimeError("No get_status reply -- check the serial link/firmware.")


def get_cmdq_stats(ser, retries=3):
    """A single query occasionally gets no reply within the deadline when
    called right after a fast burst (observed live during sweep testing,
    root cause not fully pinned down -- possibly a residual ack from the
    prior burst still in flight). Retried rather than treated as fatal, so
    one flaky query doesn't take down an otherwise-fine rate."""
    for attempt in range(retries):
        ser.reset_input_buffer()
        ser.write(b"cmdq_stats\n")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            m = CMDQ_STATS_RE.match(raw.decode(errors="replace").strip())
            if m:
                return int(m.group(1)), int(m.group(2))
        # no reply this attempt -- brief pause, then retry
        time.sleep(0.2)
    raise RuntimeError(f"No cmdq_stats reply after {retries} attempts")


def report(label, samples_s, requested_trials):
    if not samples_s:
        print(f"{label}: no successful round trips out of {requested_trials} -- check the link.")
        return
    ms = sorted(s * 1000.0 for s in samples_s)
    n = len(ms)

    def pct(p):
        idx = min(n - 1, int(round(p / 100 * (n - 1))))
        return ms[idx]

    print(f"{label}: n={n}/{requested_trials}  "
          f"min={ms[0]:.3f}ms  mean={statistics.mean(ms):.3f}ms  "
          f"median={statistics.median(ms):.3f}ms  "
          f"p95={pct(95):.3f}ms  p99={pct(99):.3f}ms  max={ms[-1]:.3f}ms")


def run_ping(ser, trials):
    samples = []
    for _ in range(trials):
        ser.reset_input_buffer()
        elapsed = _round_trip(ser, "get_status", STATUS_RE)
        if elapsed is not None:
            samples.append(elapsed)
    report("ping (get_status, wait-for-reply)", samples, trials)


def run_setpos(ser, trials):
    x = get_current_x(ser)
    print(f"Current x_center={x} -- re-sending this SAME value {trials}x (no net movement).")
    samples = []
    for _ in range(trials):
        ser.reset_input_buffer()
        elapsed = _round_trip(ser, f"set_x {x}", SET_X_ACK_RE)
        if elapsed is not None:
            samples.append(elapsed)
    report("setpos (set_x, wait-for-ack)", samples, trials)


def run_burst(ser, burst_n):
    x = get_current_x(ser)
    _, dropped_before = get_cmdq_stats(ser)
    print(f"Current x_center={x}, dropped(before)={dropped_before} -- firing "
          f"{burst_n}x `set_x {x}` back-to-back with NO waiting (true fire-and-forget).")

    line = f"set_x {x}\n".encode("ascii")
    t0 = time.monotonic()
    for _ in range(burst_n):
        ser.write(line)
    t_sent = time.monotonic() - t0

    time.sleep(0.5)  # let the firmware's main loop drain the queue before asking
    depth_after, dropped_after = get_cmdq_stats(ser)
    dropped_this_burst = dropped_after - dropped_before

    print(f"Sent {burst_n} commands in {t_sent * 1000:.1f}ms "
          f"({burst_n / t_sent:.0f} commands/sec sustained send rate).")
    print(f"Firmware queue depth after drain: {depth_after} (should be ~0 if "
          f"fully drained). Dropped this burst: {dropped_this_burst}.")
    if dropped_this_burst > 0:
        print("Some commands were dropped at this send rate -- back off or "
              "add flow control before relying on fire-and-forget this fast.")
    else:
        print("No drops -- this send rate is safe for fire-and-forget.")


def run_sweep(ser, rates_hz, n_per_rate):
    x = get_current_x(ser)
    print(f"Current x_center={x} -- sweeping paced fire-and-forget `set_x {x}` "
          f"at {len(rates_hz)} candidate rates, {n_per_rate} commands each "
          "(no net movement, always re-sending the FTA's own current position).")
    line = f"set_x {x}\n".encode("ascii")

    print(f"{'req_hz':>8} {'achieved_hz':>12} {'dropped':>8} {'drop_pct':>9}")
    results = []
    for rate in rates_hz:
        period = 1.0 / rate
        try:
            _, dropped_before = get_cmdq_stats(ser)

            t_start = time.monotonic()
            for i in range(n_per_rate):
                t_target = t_start + i * period
                now = time.monotonic()
                if t_target > now:
                    time.sleep(t_target - now)
                ser.write(line)
            t_end = time.monotonic()
            achieved_rate = n_per_rate / (t_end - t_start)

            time.sleep(0.5)  # let the firmware's main loop drain before checking
            _, dropped_after = get_cmdq_stats(ser)
        except RuntimeError as e:
            print(f"{rate:>8.0f}          ?        ?  query failed: {e}")
            continue
        dropped_this = dropped_after - dropped_before
        drop_pct = 100.0 * dropped_this / n_per_rate

        print(f"{rate:>8.0f} {achieved_rate:>12.1f} {dropped_this:>8d} {drop_pct:>8.1f}%")
        results.append((rate, achieved_rate, dropped_this))

    clean = [r for r in results if r[2] == 0]
    if clean:
        best = max(clean, key=lambda r: r[0])
        print(f"\nHighest tested rate with ZERO drops: requested {best[0]:.0f}Hz "
              f"(achieved {best[1]:.1f}Hz), n={n_per_rate} each.")
    else:
        print("\nEven the lowest tested rate showed drops -- try lower rates "
              "with --sweep-rates.")
    return results


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["ping", "setpos", "burst", "sweep"], default="ping")
    parser.add_argument("--trials", type=int, default=300)
    parser.add_argument("--burst-n", type=int, default=500)
    parser.add_argument("--sweep-rates", default="100,200,400,600,800,1000,1200,1400,1600,1800,2000,2500")
    parser.add_argument("--sweep-n", type=int, default=300)
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=460800)
    args = parser.parse_args()

    import serial

    port = args.port or find_fta_port()
    if port is None:
        print("No ST-Link serial port found -- pass --port explicitly, or "
              "check the Nucleo's USB cable is connected to this Pi.")
        raise SystemExit(1)
    print(f"Connecting to {port} @ {args.baud}")
    ser = serial.Serial(port, args.baud, timeout=1)
    time.sleep(2)  # let the Nucleo's USB-serial enumerate/settle
    ser.reset_input_buffer()

    try:
        if args.mode == "ping":
            run_ping(ser, args.trials)
        elif args.mode == "setpos":
            run_setpos(ser, args.trials)
        elif args.mode == "burst":
            run_burst(ser, args.burst_n)
        elif args.mode == "sweep":
            rates = [float(r) for r in args.sweep_rates.split(",")]
            run_sweep(ser, rates, args.sweep_n)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
