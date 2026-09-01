#!/usr/bin/env python3
"""
Directly measures the REAL achieved control-step rate under the exact
current operating conditions (whatever telemetry rate is live, whatever
ctrl_rate throttle is set), rather than trusting g_ctrl_rate_millihz's
NOMINAL/requested value -- which is what notch_configure()/lead_configure()
actually use to compute their filter coefficients.

Motivation: the control-step gate is
    (HAL_GetTick() - g_last_ctrl_step_tick) >= g_control_interval_ms
which only fires on a telemetry packet that happens to arrive AFTER the
interval has elapsed -- if the real telemetry period doesn't divide the
throttle interval evenly, the REAL achieved rate can differ from the
nominal throttle target by a real, non-trivial amount (quantization
between packet period and throttle interval), even though PID's own dt-
aware math is immune to this (it measures real elapsed time per step) --
notch/lead are NOT dt-aware, they use one fixed assumed rate.

Reuses the dac_y= telemetry field as a firing-detector (apply_dac() is
only ever called from run_closed_loop_step in closed_loop mode, so dac_y
only changes on a real control-step firing) and tick= for timing (firmware's
own HAL_GetTick(), immune to host clock jitter) -- same technique as
scratch_ctrl_jitter_check.py earlier this session.
"""
import re
import threading
import time

import numpy as np

FTA_BAUD = 460800
REPLY_RE = re.compile(r"^(OK|ERR|STATUS|WARN)\b")
TELEMETRY_RE = re.compile(
    r"^seq=\s*(\d+)\s+status=(\d+)\s+x=(-?\d+\.\d)\s+y=(-?\d+\.\d)\s+"
    r"tgt=(-?\d+\.\d)\s+dac_y=(-?\d+)\s+tick=(\d+)\s+pkts=(\d+)\s+errs=(\d+)$")
STATUS_FIELD_RE = {
    "dac_y": re.compile(r"dac_y=(-?\d+)"),
    "amp": re.compile(r"amp=(\d+)"),
    "tel_age_ms": re.compile(r"tel_age_ms=(\d+)"),
    "ctrl_rate_millihz": re.compile(r"ctrl_rate_millihz=(-?\d+)"),
    "ctrl_interval_ms": re.compile(r"ctrl_interval_ms=(\d+)"),
    "target_x_set": re.compile(r"target_x_set=(\d+)"),
}


def find_fta_port():
    from serial.tools import list_ports
    candidates = [p for p in list_ports.comports()
                  if any(t in (p.description or "") for t in ("STLink", "ST-Link", "STMicroelectronics"))]
    return candidates[0].device if candidates else None


def send_command(ser, cmd, char_delay=0.02, reply_timeout=2.0, retries=5):
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
            return {k: int(m.group(1)) for k, m in matches.items()}
    raise RuntimeError("no parseable get_status")


def main():
    import serial
    port = find_fta_port()
    print(f"connecting {port}")
    ser = serial.Serial(port, FTA_BAUD, timeout=0.2)
    time.sleep(2)
    ser.reset_input_buffer()
    send_command(ser, "clear_estop")

    st = get_status(ser)
    print(f"current: ctrl_rate_millihz={st['ctrl_rate_millihz']} "
          f"ctrl_interval_ms={st['ctrl_interval_ms']} amp={st['amp']}")

    send_command(ser, "set_mode open_loop")
    send_command(ser, "set_y 2048")
    time.sleep(0.4)

    tel_x_re = re.compile(r"tel_x=(-?[\d.]+)")
    reply = send_command(ser, "get_status")
    m = tel_x_re.search(reply or "")
    baseline_cx = float(m.group(1)) if m else 200.0
    target_from = round(baseline_cx)
    target_to = target_from - 25  # a real, continuous, nonzero error to correct
    print(f"baseline cx={baseline_cx:.1f}  step {target_from} -> {target_to}")

    amp_was_enabled = bool(st["amp"])
    if not amp_was_enabled:
        send_command(ser, "amp_enable")

    send_command(ser, f"set_target_x {target_from}")
    send_command(ser, "set_mode closed_loop")
    time.sleep(0.3)
    send_command(ser, f"set_target_x {target_to}")  # the real step -- guarantees
                                                      # continuous nonzero error
                                                      # for the whole recording,
                                                      # so dac_y changes on
                                                      # (nearly) every real firing

    records = []
    stop_event = threading.Event()

    def reader():
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
            dac_y = int(m.group(6))
            tick_ms = int(m.group(7))
            records.append((tick_ms, dac_y))

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    time.sleep(4.0)
    stop_event.set()
    t.join(timeout=1.0)

    send_command(ser, "set_mode open_loop")
    send_command(ser, "set_x 95")  # axis2 is on -- dac_x drifts too, reset it as well
    send_command(ser, "set_y 95")
    if not amp_was_enabled:
        send_command(ser, "amp_disable")
    print("idle check:", send_command(ser, "get_status"))
    ser.close()

    if len(records) < 20:
        print(f"only {len(records)} samples, not enough")
        return

    tick_all = np.array([r[0] for r in records], dtype=np.float64)
    dac_y_all = np.array([r[1] for r in records], dtype=np.int64)

    # Restrict to the active transient window only -- same fix
    # scratch_ctrl_jitter_check.py needed earlier this session: once the
    # loop settles, consecutive corrections round to the same integer
    # dac_y for many real firings in a row, inflating apparent gaps and
    # badly underestimating the real firing rate. The step was commanded
    # ~0.3s after tick[0] (after the 0.3s post-engagement settle sleep);
    # restrict to the first 0.4s after that, comfortably inside this
    # rig's established ~150-400ms Ki=200 settling range.
    step_tick_est = tick_all[0] + 300  # ms, matches the 0.3s pre-step sleep above
    window_end = step_tick_est + 400  # ms
    mask = (tick_all >= step_tick_est) & (tick_all <= window_end)
    tick = tick_all[mask]
    dac_y = dac_y_all[mask]
    print(f"\nrestricted to active transient window: {mask.sum()} of {len(records)} samples")

    # telemetry packet rate (raw, includes non-firing packets too)
    total_span_s = (tick[-1] - tick[0]) / 1000.0
    telemetry_hz = len(records) / total_span_s if total_span_s > 0 else float("nan")

    # real control-step firings: dac_y value actually changed since last sample
    changed = np.where(np.diff(dac_y) != 0)[0] + 1
    fire_ticks = tick[changed]
    if len(fire_ticks) < 5:
        print(f"only {len(fire_ticks)} dac_y transitions detected -- "
              "too few to measure a real rate (try a bigger step, or check amp/mode).")
        print(f"raw telemetry rate: {telemetry_hz:.1f}/s over {total_span_s:.2f}s, "
              f"{len(records)} samples")
        return

    intervals_ms = np.diff(fire_ticks)
    real_rate_hz = 1000.0 / np.mean(intervals_ms)

    print(f"\nraw telemetry rate (all packets): {telemetry_hz:.1f}/s "
          f"({len(records)} samples over {total_span_s:.2f}s)")
    print(f"real control-step firings detected: {len(fire_ticks)}")
    print(f"real achieved control-step interval: mean={np.mean(intervals_ms):.2f}ms "
          f"median={np.median(intervals_ms):.2f}ms std={np.std(intervals_ms):.2f}ms "
          f"min={np.min(intervals_ms):.1f}ms max={np.max(intervals_ms):.1f}ms")
    print(f"real achieved control-step rate: {real_rate_hz:.1f}Hz")
    print(f"\nnominal/configured ctrl_rate_millihz was: {st['ctrl_rate_millihz']/1000.0:.1f}Hz "
          f"(this is what notch_configure/lead_configure actually use)")
    mismatch_pct = (real_rate_hz - st['ctrl_rate_millihz']/1000.0) / (st['ctrl_rate_millihz']/1000.0) * 100
    print(f"mismatch: {mismatch_pct:+.1f}%")


if __name__ == "__main__":
    main()
